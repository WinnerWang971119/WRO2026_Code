# Operator Diagnostics Console Plan

**Issue:** [#166](https://github.com/WinnerWang971119/WRO2026_Code/issues/166) — *operator menu: add port view, vision mode, and a mode-selectable (SYNC/GYRO/RAW) move test*

**Goal:** Replace the 3-item pre-arm calibration menu with a read-only on-OLED diagnostics console so a competition-day operator can verify the robot's sensors and power are fit to run — at the venue, in minutes, with no laptop and no reflashing.

**Approach:** Extract the console out of `WRO2026.ino` into a new upper-layer `src/operator/` domain, split into an **OLED-free core** (`OperatorConsole`) and a thin **hardware adapter** (`OperatorPanel`). The core owns the item table, the scroll/confirm FSM, button edge detection, the live-view states and the SELF TEST sequencer; it consumes a pushed-in sample POD and emits a 4-row text frame plus LED/buzzer intents and an action request. The adapter pushes cheap cached state on a cadence and executes every expensive or latching read on the core's request. The core links into the host ctest harness; the adapter does not. Every screen the console owns is per-tick and non-blocking.

## Scope

- **In scope:**
  - New `src/operator/` domain: `ConsoleTypes.h` (PODs), `OperatorConsole.h/.cpp` (headless core), `OperatorPanel.h/.cpp` (Arduino adapter).
  - Nine menu entries on a flat scrolling list: `SELF TEST`, `BATT`, `LINE VIEW`, `VISION VIEW`, `GYRO VIEW`, `GYRO ZERO`, `CAL LINE`, `CAL MOTOR`, `EXIT`.
  - Three live views (LINE / VISION / GYRO) plus the BATT screen — per-tick, exit on BTN_DOWN, no timeout.
  - `SELF TEST`: battery → gyro → vision → line, **halting on the first failure**, verdict on LEDs + buzzer, advisory only.
  - New `WROLineArray::probeBoard()` — a side-effect-free I²C address-ACK presence probe (see Decisions).
  - `GYRO ZERO`: dispatches `YawSource.zeroHardware()`.
  - Replacing both blocking release-spins with a non-blocking `WAIT_RELEASE` state.
  - Capturing the 13 boot health booleans into a persistent `BootReport` POD (today they are `setup()` locals that die immediately).
  - **Boot-failure degraded mode.** The console stays openable when `setup()` fails its health checks (`_setupFailed`): read-only screens remain available so the operator can read the boot report about the failure that just occurred, while mission arming and every hardware-writing entry stay disabled. `_setupFailed` is latched for the power cycle and no console entry may clear it.
  - `.ino` integration: line-array demand OR, EventTrace suppression, boot-report capture, the narrowed `_setupFailed` guard, removal of the old menu.
  - New `config/ConfigUi.h` + `ConfigChecks.h` static asserts.
  - Host ctest coverage for the core; `ADR-076`; `DECISIONS.md` + `ARCHITECTURE.md` updates.

- **Out of scope:**
  - **Any entry that moves the robot.** The mode-selectable SYNC/GYRO/RAW move test named in #166 is explicitly dropped — user decision, this console is read-only. `CAL MOTOR` is the sole exception and is pre-existing behaviour, unchanged.
  - **A raw I²C / mux port-scan entry.** Rejected by the user ("the vendor I2C sucks"). Note `probeBoard()` is *not* this: it is an internal, fixed-address, two-board presence check with no operator-facing scan UI.
  - **Making `WROLineArray::calibrate()` or `Drive::calibrateMotorBias()` non-blocking.** Both freeze the cooperative loop today and the console changes neither. Converting `calibrate()` to an FSM means restructuring a two-phase operator-paced sweep with a 60 s button gate — a `WROLineArray` refactor with its own test surface, not console work. Tracked separately.
  - Live constant tuning, EEPROM persistence, post-run fault forensics, tick-rate/encoder screens.
  - Retiring or modifying any `inoTest/` sketch.
  - Changing `Mission`, `PathTracker`, or any drive behaviour. The mission-arm path's *behaviour* is unchanged; only its guard moves (T7) so `_setupFailed` suppresses arming without also suppressing the console.

## Decisions

- **Domain is `src/operator/`, not `src/util/`** — `src/util/` is the *bottom* layer (`WROMath`, `TickProfiler`, `EncoderSource` all sit below `Drive`). A console that samples `WROLineArray`, `Vision` and `YawSource`, and commands `Drive` (via `CAL_MOTOR`) and `WROLineArray` (via `CAL_LINE`), is a top-layer peer of `Mission`; `util/` would invert the dependency arrow. Both approach consultants raised this independently.
  *(Correction applied during execution: the console does not read `Localization` at all — it appears only as a `BootReport` flag captured by the sketch. `Drive` is command-only; there is no `Drive` field in `DiagSample`.)*
- **Core + adapter split** — user decision, after both consultants confirmed a single OLED-owning `.cpp` cannot enter the host harness. `WRO2026/test/CMakeLists.txt:360-364` documents that no real `EventTrace.cpp` is linked for the force-included path targets, to dodge the `MiniR4OLED.h` `WHITE`-macro collision; no target links it. A monolithic `OperatorMenu.cpp` would inherit that exclusion and buy zero CI coverage.
- **Hybrid data flow: adapter pushes cheap cached state; the core pulls every expensive or latching read.** Pure adapter-push would delete the only mechanism that can order a fresh read before SELF TEST judges, and would fire the ~240 ms blocking `readMosaic()` on a free-running cadence. Pure core-pull would force pointless round-trips for already-cached state. Every `DiagSample` field has exactly one initiator and one authoritative channel (see Contracts → Authority table).
- **Three data paths for line, each where it is correct** — (1) `probeBoard()` is **authoritative for the board-answered verdict**. (2) `readBoard()` is authoritative for raw channel display and `0xFF`-class transport corruption **only** — it is *not* a liveness signal. (3) The forced-open tick gate feeds the derived telemetry (`getJunctionType()`, `isOnline()`, `getError()`), which are pure cache readers when `LINE_ARRAY_TICK_CACHE_ENABLED` is on.
- **Board liveness is proven by an I²C address ACK, never by channel values.** `WROLineArray::_readBoardInternal()` states outright that `MatrixColor::getColor()` has no error return, that a bus failure surfaces as a `0` byte indistinguishable from a saturated off-line reading, and that the code "accept[s] the read at face value and return[s] true". So `readBoard()` returning `true` does **not** prove the board answered. The vendor's own `MatrixColor::begin()` proves presence via `Device_ID == 0x43`, but it also resets `Device_CONFIG` and blocks ~550 ms, which would wipe the Layer-1 illumination latched in `WROLineArray::begin()`. Hence a new side-effect-free `probeBoard()`: mux-select (`ADDR_PCA954X`, `1 << _ch`), then `beginTransmission(MatrixColor_ADDR)` / `endTransmission() == 0`. No register write, no value inspection, ~1 ms. Both addresses are vendor `#define`s. *Rejected alternatives:* an all-zero detector (judges values, and false-fails — a bar held off the mat legitimately reads all-zero, and on Renesas an empty RX buffer returns `-1` → `0xFF`, already caught by the `> 99` guard); two-reads-differ (`0..9` quantisation makes a live stationary sensor routinely bit-identical → mass false-fails).
- **Frame geometry is not a Config tunable.** It is a structural property of `ConsoleFrame` and lives solely in `ConsoleTypes.h`. `config/` must never include a `src/` header — `Config.h` is transitively included by every module, so a `ConfigChecks.h` → `ConsoleTypes.h` include would inject a `src/` header into every TU. `ConfigUi.h` keeps only timing/threshold tunables.
- **Per-tick FSM, no blocking view loops** — a no-timeout view implemented as a blocking spin is an *unbounded* operator hold. Dispatched **actions** stay sanctioned-blocking; **views** are FSM states. Note the sanctioned sites are not uniform: `Drive::_drainBlocking()` and `Vision::readMosaic()` spin the registered tick callback, while the two calibration routines simply block. That asymmetry is why AC4's guarantee stops at the dispatch boundary.
- **Release-spins replaced with `WAIT_RELEASE`** — user decision. The current `while (pressed) delay(20)` spins do **not** call `cooperativeTick()`, so every module freezes while a button is held.
- **SELF TEST halts on the first failure** — user decision, upheld against Fable's veto. Fable argued for a 4-row all-checks verdict "exactly the panel's capacity"; that miscounts — the panel is 4 rows *total* and row 0 is the title, leaving 3. Codex caught the error and defended halt-on-first.
- **SELF TEST judges only freshly-refreshed data, and fails closed.** `Vision::begin()` clears any latched fault, so a boot-state `Fault::NONE` would otherwise PASS the vision check with the camera unplugged. Every check latches `sample.seq` at entry and judges only a sample whose `seq` advanced after that; a check whose data does not arrive within `UI_SELFTEST_CHECK_TIMEOUT_MS` **FAILs**. `UI_SELFTEST_STEP_MS` is a minimum dwell for readability, never a deadline.
- **Degraded mode is per-subsystem, not all-or-nothing** — a boot failure is exactly when the operator needs the report, so the console must open; but a module whose `begin()` returned `false` must not be sampled or commanded. Availability derives purely from the `BootReport` the core already receives, so it stays host-testable. Note `boot.batt` is a *voltage verdict*, not an init verdict (`MiniR4.PWR` comes up in `MiniR4.begin()`), so a low pack never disables the BATT screen — the low-battery-only boot failure is the most common degraded case and the one where diagnostics matter most.
- **SELF TEST is advisory only** — it never gates or warns on the BTN_UP arm path.
- **Vision check judges link health only** — `READ_TIMEOUT` / `BAD_CHECKSUM` = FAIL; `BAD_CELL` = **PASS**. `BAD_CELL` fires when a cell classifies `UNKNOWN`, the expected pre-run state. Judging cell content would transplant the mistake already vetoed for line channels.
- **`CAL MOTOR` is retained but cautioned** *(decided under `--auto`; the single decision most worth an override)*. Codex rated removal critical: it is an actuator command in a console whose stated goal is read-only, and its output is known-wrong while the M2/M3 encoder cables are crossed (it compares wheels differentially, so it assigns each wheel's bias to the other). Retained because it is pre-existing behaviour this issue did not ask to change, and it is currently inert (bias 1.0/1.0). It gets an on-screen caution row and is non-dispatchable whenever the boot report is unclean.
- **Menu entry, gating and title** — entry stays BTN_DOWN long-press ≥ 2 s, pre-arm only. Title renames `CALIBRATION` → `DIAGNOSTICS`.
- **Nothing is persisted** — no EEPROM. Calibration output still goes to Serial for paste-back into `Config.h`.

*Assumptions decided under `--auto` (the Phase 3 batch went unanswered): title rename to `DIAGNOSTICS`; `CAL MOTOR` retained; no persistence; SELF TEST order batt→gyro→vision→line; GYRO VIEW includes a drift watch; VISION VIEW re-reads on a cadence (throttled to `UI_VISION_SAMPLE_MS`, since `readMosaic()` blocks ~240 ms); BATT carries the boot health list.*

## Acceptance Criteria

- [ ] **AC1** — A BTN_DOWN long-press ≥ `UI_MENU_LONGPRESS_MS` while un-armed opens a `DIAGNOSTICS` list showing the title plus 3 item rows, cursor on `SELF TEST`.
- [ ] **AC2** — BTN_DOWN cycles the cursor through all 9 entries in order and wraps; the visible 3-row window scrolls to keep the cursor on screen at both ends.
- [ ] **AC3** — BTN_UP confirms the highlighted entry and dispatches it.
- [ ] **AC4** — No code path *owned by the console* calls `delay()` or spins while holding the cooperative tick: the menu FSM, every view, the SELF TEST sequencer, `WAIT_RELEASE` and frame rendering are per-tick and return within one tick. `cooperativeTick()` continues at its normal cadence with the menu open, with any view open, and while a button is held. **Excluded:** the two dispatched pre-existing calibration actions, `CAL LINE` (`WROLineArray::calibrate()`) and `CAL MOTOR` (`Drive::calibrateMotorBias()`). Neither spins a tick callback — `WROLineArray` exposes no `setTickCallback` and `calibrateMotorBias()` uses a bare `delay()` — so the loop *is* frozen for their duration (up to ~70 s for `CAL LINE` on a 2-board config). This is unchanged from today's menu; the console must not make it worse and must return to per-tick operation the moment `notifyActionDone()` arrives.
- [ ] **AC5** — While LINE VIEW is open or SELF TEST is at `Check::LINE`, `lineArrayNeededThisTick()` returns `true`, and LINE VIEW's derived fields change in response to physically moving the robot over a line.
- [ ] **AC6** — LINE VIEW shows 8 raw channel values per configured board sourced from `readBoard()`, and renders `BOARDn: NO DATA` for a board whose read failed, substituting no zeros.
- [ ] **AC7** — VISION VIEW shows 12 cells as colour letters (`? Y B G W R`) refreshing on `UI_VISION_SAMPLE_MS`, plus the current `VisionClass::Fault`. After leaving the view, `Vision.hasFault()` is `false`.
- [ ] **AC8** — GYRO VIEW shows current yaw, `isHealthy()`, and wrap-correct drift since the view opened; leaving and re-entering resets the drift baseline to zero.
- [ ] **AC9** — GYRO ZERO emits `ActionRequest::GYRO_ZERO`, the adapter calls `YawSource.zeroHardware()`, and the console returns to the list.
- [ ] **AC10** — BATT shows pack voltage, percentage, and a per-module boot OK/FAIL summary reflecting the health results captured in `setup()`.
- [ ] **AC11** — SELF TEST runs battery → gyro → vision → line in that order and **stops at the first FAIL**, naming the failed subsystem on screen.
- [ ] **AC12** — SELF TEST all-pass drives green LEDs + the boot-OK chirp pattern; any failure drives red LEDs + the boot-FAIL long tone. The chirp sequence is timestamp-driven, never `delay()`-driven.
- [ ] **AC13** — The SELF TEST line check passes iff `LineArray.probeBoard(b)` returns `true` for every board `b < Config::LINE_BOARD_COUNT`. It never inspects channel values and never uses `readBoard()`'s return as a liveness signal.
- [ ] **AC13b** — `WROLineArray::probeBoard(uint8_t)` performs an I²C address ACK against the board's mux channel and returns `endTransmission() == 0`. It writes no device register, issues no `setLight`/`begin` call, and leaves `_lastChannels` / `_lastOnline` untouched.
- [ ] **AC14** — The SELF TEST vision check runs only after a `READ_MOSAIC` it dispatched has completed; it FAILs on `READ_TIMEOUT`/`BAD_CHECKSUM` and PASSes on `BAD_CELL`/`NONE`. A `NONE` never preceded by a completed read is **never** a PASS.
- [ ] **AC15** — A SELF TEST failure never blocks, delays or warns on the BTN_UP arm path.
- [ ] **AC16** — BTN_DOWN exits any view back to the list. No view has a timeout.
- [ ] **AC17** — The console's OLED ownership is exclusive while open, and `EventTrace`'s accumulated events repaint on the first tick after it closes. EventTrace's Serial heartbeat continues to emit while the console is open.
- [ ] **AC18** — The console is unavailable once the mission is armed; BTN_DOWN retains its fault-acknowledge behaviour.
- [ ] **AC19** — `OperatorConsole` compiles and links in the host ctest harness with no `<MatrixMiniR4.h>` / `<Arduino.h>` dependency, and the new tests pass in CI.
- [ ] **AC20** — `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` succeeds.
- [ ] **AC21** — While the console is closed, `OperatorPanel.update()` issues no I²C transaction, no `readBoard()`/`probeBoard()`, no `Vision` read and no OLED write; the mission's tick cadence is identical with the panel present and absent.
- [ ] **AC22** — Every SELF TEST check judges only a sample refreshed after that check began (`seq` advanced, plus `notifyReadDone()` for vision and line). A check whose fresh data does not arrive within `UI_SELFTEST_CHECK_TIMEOUT_MS` FAILs and halts.
- [ ] **AC23** — After a boot failure (`_setupFailed == true`), a BTN_DOWN long-press still opens `DIAGNOSTICS`, and BATT names exactly the module(s) that failed. BTN_UP never arms for the whole power cycle.
- [ ] **AC24** — Entry availability derives from `BootReport`. An entry whose owning module reported `begin() == false` renders with a `!` marker and is non-dispatchable. `CAL LINE`, `CAL MOTOR` and `GYRO ZERO` are additionally non-dispatchable whenever the boot report is unclean. `BATT`, `SELF TEST` and `EXIT` are always dispatchable. A `boot.batt == false` never disables a read-only entry.
- [ ] **AC25** — In degraded mode no `DiagSample` field is sampled from a module whose boot flag is `false`; the adapter leaves the read-failed sentinel and the view renders `--`. `wantsLineData()` returns `false` when `boot.lineArray == false`. SELF TEST reports such a subsystem straight from the boot report, issuing no `ActionRequest` against it.

## Contracts & Interfaces

### Shared types — `WRO2026/src/operator/ConsoleTypes.h` (owner: T2)

`<stdint.h>` only. No `<Arduino.h>`, no `<MatrixMiniR4.h>`, no `Config.h`.

```cpp
namespace Console
{
    enum class Item : uint8_t
    {
        SELF_TEST = 0, BATT = 1, LINE_VIEW = 2, VISION_VIEW = 3,
        GYRO_VIEW = 4, GYRO_ZERO = 5, CAL_LINE = 6, CAL_MOTOR = 7, EXIT = 8
    };
    constexpr uint8_t ITEM_COUNT = 9;

    enum class State : uint8_t
    {
        CLOSED = 0, LIST = 1, VIEW_LINE = 2, VIEW_VISION = 3,
        VIEW_GYRO = 4, VIEW_BATT = 5, SELF_TEST = 6, WAIT_RELEASE = 7
    };

    /** @brief Side-effect actions (report via notifyActionDone) and reads (via notifyReadDone). */
    enum class ActionRequest : uint8_t
    {
        NONE = 0, CAL_LINE = 1, CAL_MOTOR = 2, GYRO_ZERO = 3,
        READ_MOSAIC = 4, READ_LINE_BOARDS = 5, CLEAR_VISION_FAULT = 6
    };

    enum class Signal : uint8_t { NONE = 0, PASS = 1, FAIL = 2 };
    enum class Check  : uint8_t { BATT = 0, GYRO = 1, VISION = 2, LINE = 3, DONE = 4 };

    /** @brief The 13 boot health results captured once in setup(). */
    struct BootReport
    {
        bool  batt;          // voltage verdict, NOT an init verdict
        bool  lineArray, yaw, drive, lineFollower, intersection, manipulator,
              localization, pathTracker, mission, tasks, vision, trace;
        float battVoltageAtBoot;

        bool clean() const;  // AND of all 13 flags
    };

    /**
     * @brief Hardware state pushed to the core.
     * PUSHED group: refreshed autonomously by the adapter every UI_VIEW_SAMPLE_MS.
     * PULLED group: written ONLY as the completion of the matching ActionRequest;
     *               otherwise carried forward byte-identical. The adapter must never
     *               refresh a PULLED field on its own.
     */
    struct DiagSample
    {
        uint32_t seq;                 // monotonic; bumped on every hardware refresh

        // --- PUSHED ---
        float    battVolts;           // 0.0f => read failed
        uint8_t  battPercent;
        bool     battReadOk;
        bool     battRetryPending;    // retry in flight; batt verdict must wait
        double   gyroYawDeg;
        bool     gyroHealthy;
        bool     gyroReadOk;
        uint8_t  junctionType;
        bool     lineOnline;
        float    lineError;

        // --- PULLED by READ_LINE_BOARDS ---
        uint8_t  lineRaw[2][8];       // 0..9 per channel
        bool     lineBoardPresent[2]; // probeBoard() ACK — THE liveness verdict
        bool     lineBoardOk[2];      // readBoard() return — raw-row validity for DISPLAY only;
                                      // true does NOT prove the board answered
        uint8_t  lineBoardCount;

        // --- PULLED by READ_MOSAIC ---
        uint8_t  visionCell[12];      // static_cast<uint8_t>(Config::Color)
        uint8_t  visionFaultKind;     // static_cast<uint8_t>(VisionClass::Fault)
    };

    struct ConsoleInput
    {
        bool       btnUp;
        bool       btnDown;
        uint32_t   nowMs;
        bool       missionArmed;
        DiagSample sample;
        BootReport boot;
    };
    // ConsoleInput carries no open-request field; opening is an out-of-band latched
    // call on the core, like notifyActionDone().

    constexpr uint8_t FRAME_ROWS        = 4;              // 128x32 @ textSize 1
    constexpr uint8_t TEXT_COLS         = 21;             // printable chars per row
    constexpr uint8_t FRAME_COLS        = TEXT_COLS + 1;  // + NUL
    constexpr uint8_t LIST_VISIBLE_ROWS = FRAME_ROWS - 1; // rows left after the title

    static_assert(FRAME_COLS == TEXT_COLS + 1, "row buffer must hold TEXT_COLS chars + NUL");
    static_assert(ITEM_COUNT >= LIST_VISIBLE_ROWS, "scroll window cannot exceed the item table");

    struct ConsoleFrame
    {
        char          rows[FRAME_ROWS][FRAME_COLS];
        uint8_t       cursorRow;   // 0xFF = no cursor
        bool          dirty;       // level-triggered until notifyFrameRendered()
        ActionRequest action;
        Signal        signal;      // level-triggered until notifySignalDone()
    };

    /** @brief AC24 availability predicate. Pure; host-tested. */
    bool isDispatchable(Item item, const BootReport& boot);
}
```

### Signatures

- `void OperatorConsoleClass::begin()` — owner: T3; consumers: T6.
- `void OperatorConsoleClass::open()` — owner: T3; consumers: T6. Latches a one-shot open request the *next* `update()` consumes: `CLOSED` → `LIST`, cursor reset to `Item::SELF_TEST`. No-op when already open. The latch clears on consumption and on force-close, so a held button cannot re-open or reset a live view. `in.missionArmed == true` discards the latch without opening.
- `void OperatorConsoleClass::update(const Console::ConsoleInput& in, Console::ConsoleFrame& out)` — owner: T3; consumers: T6, T8. **Pure:** no I/O, no `millis()`, no statics outside the instance. `in.nowMs` is the only time source.
- `void OperatorConsoleClass::notifyActionDone(Console::ActionRequest which, bool ok)` — owner: T3; consumers: T6. **Side-effect actions only** (`CAL_LINE`, `CAL_MOTOR`, `GYRO_ZERO`, `CLEAR_VISION_FAULT`); `ok` is the sole authority on success.
- `void OperatorConsoleClass::notifyReadDone(Console::ActionRequest which)` — owner: T3; consumers: T6, T8. **`READ_MOSAIC` / `READ_LINE_BOARDS` only**; carries no verdict. The refreshed PULLED fields are the sole authority. The adapter must refresh `DiagSample` (bumping `seq`) *before* calling this.
- `void OperatorConsoleClass::notifyFrameRendered()` / `notifySignalDone()` — owner: T3; consumers: T6. Clear `dirty` / `signal`; both are level-triggered until acknowledged so a suppressed repaint or an unfinished chirp is never lost.
- `bool OperatorConsoleClass::isOpen() const` — owner: T3; consumers: T6, T7.
- `bool OperatorConsoleClass::wantsLineData() const` — owner: T3; consumers: T6, T7. `true` in `VIEW_LINE`, and at `Check::LINE` from the tick `READ_LINE_BOARDS` is emitted until the line verdict is recorded. `false` whenever `boot.lineArray == false`. Gates the PUSHED line-derived fields and `LineArray` ticking only; never authorises a raw `readBoard()`.
- `Console::State OperatorConsoleClass::state() const` — owner: T3; consumers: T8.
- `bool WROLineArrayClass::probeBoard(uint8_t boardIdx)` — owner: T5; consumers: T6.
- `bool OperatorPanelClass::begin()` / `void open()` / `void update()` / `bool isOpen() const` / `bool wantsLineData() const` / `void setBootReport(const Console::BootReport&)` / `void setMissionArmed(bool)` — owner: T6; consumers: T7. `open()` delegates to `OperatorConsole.open()` and holds no open state. `update()` **returns immediately when closed** (button-level read only).
- `extern OperatorConsoleClass OperatorConsole;` / `extern OperatorPanelClass OperatorPanel;` — iron rule 2.

### Authority table

| Field group | Initiator | Authoritative channel |
|---|---|---|
| `battVolts`/`battPercent`/`battReadOk`/`battRetryPending` | adapter cadence | `DiagSample` |
| `gyroYawDeg`/`gyroHealthy`/`gyroReadOk` | adapter cadence | `DiagSample` |
| `junctionType`/`lineOnline`/`lineError` | adapter cadence (gated on `wantsLineData()`) | `DiagSample` |
| `lineRaw`/`lineBoardPresent`/`lineBoardOk`/`lineBoardCount` | core, via `READ_LINE_BOARDS` | `DiagSample` after `notifyReadDone` |
| `visionCell`/`visionFaultKind` | core, via `READ_MOSAIC` | `DiagSample` after `notifyReadDone` |
| `CAL_LINE`/`CAL_MOTOR`/`GYRO_ZERO`/`CLEAR_VISION_FAULT` | core | `notifyActionDone(which, ok)` |

### Adapter tick order (invariant, owner T6)

Poll buttons → **if closed, return** → refresh the PUSHED group if `UI_VIEW_SAMPLE_MS` elapsed (bump `seq`) → if a request pended from the previous tick, execute it, write its PULLED fields, bump `seq`, then call `notifyReadDone()`/`notifyActionDone()` → `OperatorConsole.update(in, frame)` → render if `dirty` and `UI_REPAINT_INTERVAL_MS` elapsed, then `notifyFrameRendered()` → drive `signal`, `notifySignalDone()` when the pattern completes → latch `frame.action` as pending.
**Invariant: at most one request in flight; when a `notify*` arrives the very next `update()` already sees that request's result.**

### File ownership

| File | Owner task | Consumer tasks |
|---|---|---|
| `WRO2026/config/ConfigUi.h`, `ConfigChecks.h`, `Config.h` | T1 | T3, T6 |
| `WRO2026/src/operator/ConsoleTypes.h` | T2 | T3, T6, T8 |
| `WRO2026/src/operator/OperatorConsole.h/.cpp` | T3 | T4, T6, T8 |
| `WRO2026/src/perception/WROLineArray.h/.cpp` | T5 | T6 |
| `WRO2026/src/operator/OperatorPanel.h/.cpp` | T6 | T7 |
| `WRO2026/WRO2026.ino` | T7 | — |
| `WRO2026/test/test_operator_console.cpp` | T8 | — |
| `WRO2026/test/CMakeLists.txt` | T9 | — |
| `docs/decisions/076-*.md`, `DECISIONS.md`, `ARCHITECTURE.md` | T10 | — |

**T3 and T4 both edit `OperatorConsole.cpp`** — T4 is blocked by T3, not parallel with it.
**`WRO2026/test/CMakeLists.txt` is owned solely by T9.**

### Naming

- Title `DIAGNOSTICS`; labels in order `SELF TEST`, `BATT`, `LINE VIEW`, `VISION VIEW`, `GYRO VIEW`, `GYRO ZERO`, `CAL LINE`, `CAL MOTOR`, `EXIT`.
- Cursor prefix `"> "`, non-cursor `"  "`, non-dispatchable `"! "`.
- Vision letters by `Config::Color`: `?` `Y` `B` `G` `W` `R`.
- `EventTrace.pushEvent`: `"DIAG MENU"`, `"SELF TEST"`, `"ST PASS"`, `"ST FAIL"`, `"GYRO ZERO"`, `"CAL LINE"`, `"CAL MOTOR"`, `"DIAG EXIT"` (≤ 19 chars; `EventTraceClass::MAX_LINE_LEN` is 20 incl. NUL).

## Data Model

`WRO2026/config/ConfigUi.h` (owner: T1), included from `Config.h` **before** `ConfigChecks.h`. Timing/thresholds only — no geometry.

```cpp
namespace Config
{
    constexpr uint32_t UI_REPAINT_INTERVAL_MS       = 200;   // OLED shares Wire1 with line + gyro
    constexpr uint32_t UI_VIEW_SAMPLE_MS            = 250;   // PUSHED-group cadence
    constexpr uint32_t UI_VISION_SAMPLE_MS          = 750;   // readMosaic() blocks up to ~240 ms
    constexpr uint32_t UI_MENU_LONGPRESS_MS         = 2000;  // unchanged from today
    constexpr float    UI_GYRO_DRIFT_WARN_DEG       = 2.0f;  // drift flagged in GYRO VIEW
    constexpr uint32_t UI_SELFTEST_STEP_MS          = 300;   // MINIMUM post-verdict dwell
    constexpr uint32_t UI_SELFTEST_CHECK_TIMEOUT_MS = 1500;  // per-check freshness deadline; FAIL on expiry
    constexpr uint32_t UI_BATT_RETRY_MS             = 50;    // setup() I2C settle, now non-blocking
    constexpr uint8_t  UI_BATT_RETRY_ATTEMPTS       = 2;     // initial read + one retry
}
```

`ConfigChecks.h` gains **Config-only** asserts (it must not reference `Console::` and must not include any `src/` header): `UI_REPAINT_INTERVAL_MS >= 100`; `UI_VISION_SAMPLE_MS > UI_VIEW_SAMPLE_MS`; `UI_SELFTEST_CHECK_TIMEOUT_MS > UI_SELFTEST_STEP_MS`; `UI_SELFTEST_CHECK_TIMEOUT_MS > UI_BATT_RETRY_MS * UI_BATT_RETRY_ATTEMPTS + 2 * UI_VIEW_SAMPLE_MS`.

## Error Handling

- **Console closed:** `OperatorPanelClass::update()` returns on its **very first statement — before even the button-level read.** `MiniR4.BTN_*.getState()` is not a GPIO read; it is an `mmL` SoftwareSerial round-trip with a 100 ms `WaitData` busy-spin worst case, so polling it while closed would add two discarded transactions to every mission `loop()` pass against a measured ~8 ms tick, breaking AC21's cadence clause and TC33. The core seeds its own `_btnUpPrev`/`_btnDownPrev` from `in.btnUp`/`in.btnDown` on the opening tick, so the adapter owes no cross-tick button continuity while closed. The adapter is the outer gate; the core's `missionArmed` force-close is the inner one. `update()` is called from `loop()` every iteration including mid-run, so a core-side-only guard would sample first and gate second. *(Corrected during execution — this originally read "after the button-level read", which T6 faithfully implemented.)*
- **Battery read fails:** retry once, **non-blocking** — on a `0.0f` read the adapter schedules a re-read `UI_BATT_RETRY_MS` later and publishes `battReadOk = false, battRetryPending = true`; after `UI_BATT_RETRY_ATTEMPTS` it latches pending false. This is the `setup()` idiom with `delay(50)` converted to a `millis()` deadline, because the console runs it from an `update()` path. SELF TEST holds in `Check::BATT` while pending; still 0 → **FAIL**. BATT screen shows `--.- V`.
- **A SELF TEST check cannot settle:** bounded by `UI_SELFTEST_CHECK_TIMEOUT_MS` from check entry. On expiry the check **FAILs** naming the subsystem (`BATT NO READ`), never PASS and never a stall. The timeout exceeds one retry plus two sample periods, so a healthy-but-slow read is never failed spuriously.
- **`probeBoard()` returns false for any configured board:** SELF TEST line check FAILs and halts, naming the board. This is the only line liveness verdict.
- **`readBoard()` returns false:** LINE VIEW shows `BOARDn: NO DATA` and leaves the raw row blank; never substitute zeros. This never drives the SELF TEST verdict — a `true` return does not prove the board answered, and a `false` return with `probeBoard()` true is transient corruption.
- **Gyro read fails or `isHealthy()` false:** SELF TEST gyro FAILs and halts. GYRO VIEW shows `YAW --` and holds the last drift baseline rather than re-baselining on a bad sample.
- **`Vision::readMosaic()` fails:** `READ_TIMEOUT`/`BAD_CHECKSUM` → FAIL; `BAD_CELL` → PASS. On any failed read the old latch is retained, so the console **clears the fault between diagnostic retries** and always emits `CLEAR_VISION_FAULT` before returning to the list — an advisory diagnostic must never leave a latched fault for the mission.
- **Both buttons held on entry or confirm:** the FSM enters `WAIT_RELEASE` and takes no action until both read released for one tick.
- **A dispatched blocking action (`CAL LINE`/`CAL MOTOR`) is in flight:** the core sits in `WAIT_RELEASE` with a pending-action flag and ignores button edges until `notifyActionDone()`. Neither routine spins a tick callback (verified: `WROLineArray` has no `setTickCallback`; `calibrateMotorBias()` uses a bare `delay()`), so the whole loop is stalled for their duration. Pre-existing behaviour, explicitly outside AC4. `Vision::readMosaic()` *does* spin the tick callback and is the only console-dispatched blocking site that keeps the loop alive.
- **Boot failed (`_setupFailed`):** the console opens; the arm path does not. Availability and sampling degrade per AC24/AC25. The red-LED boot-fail signal is left alone; a SELF TEST verdict may repaint the LEDs, and the BATT boot report remains the durable record.
- **Mission armed:** `update()` force-closes to `CLOSED` on `in.missionArmed == true`.
- **Frame row overflow:** every row uses a bounded formatter truncating at `Console::TEXT_COLS`, always NUL-terminated. No unbounded `sprintf`.

## Testing Strategy

**Levels:** Host C++ unit (ctest, CI) for the core; `arduino-cli compile` for the adapter, `probeBoard()` and sketch; manual on-hardware for I/O-bound behaviour.

| ID | Test Case | Type | Expected Behavior |
|---|---|---|---|
| TC1 | Item table order and labels | Unit | 9 labels in documented order; `ITEM_COUNT == 9` |
| TC2 | BTN_DOWN cycles and wraps | Unit | Cursor 0→8 then wraps to 0 |
| TC3 | Scroll window at both ends | Unit | Cursor 0 → items 0–2; cursor 8 → items 6–8; cursor always visible |
| TC4 | Rendered rows are exact | Unit | Cursor 0 → `rows[0]=="DIAGNOSTICS"`, `rows[1]=="> SELF TEST"`, `rows[2]=="  BATT"`, `rows[3]=="  LINE VIEW"` |
| TC5 | Button edge detection | Unit | A level held 3 ticks produces exactly one cursor advance |
| TC6 | `WAIT_RELEASE` blocks re-trigger | Unit | Confirm with both held → no second action until both release for one tick |
| TC7 | Every item dispatches correctly | Unit | Confirm on each of the 9 items yields its documented state/action; GYRO ZERO emits `ActionRequest::GYRO_ZERO` then returns to `LIST` |
| TC8 | No view timeout | Unit | 10 000 ticks, no input → still `VIEW_LINE` |
| TC9 | `wantsLineData()` truth table | Unit | `true` in `VIEW_LINE` and at `Check::LINE` after `READ_LINE_BOARDS`; `false` in `CLOSED`, `VIEW_GYRO`, and whenever `boot.lineArray == false` |
| TC10 | SELF TEST order | Unit | All healthy → BATT→GYRO→VISION→LINE then `Check::DONE` |
| TC11 | Halt on first failure | Unit | Gyro unhealthy → stops at `Check::GYRO`; vision and line never run |
| TC12 | Verdict signal | Unit | All-pass → `Signal::PASS`; any fail → `Signal::FAIL` |
| TC13 | Line check uses presence, not values | Unit | `lineBoardPresent={true,true}` + all-zero `lineRaw` + `lineBoardOk={false,false}` → **PASS**; `lineBoardPresent={true,false}` + healthy raw + `lineBoardOk={true,true}` → **FAIL** |
| TC13b | Unconfigured board ignored | Unit | `lineBoardCount=1`, `lineBoardPresent={true,false}` → PASS |
| TC13c | LINE VIEW raw-row validity | Unit | `lineBoardOk[1]==false` → board 1 row renders `NO DATA`, no zeros; SELF TEST verdict unaffected |
| TC14 | Vision fault classification | Unit | After a completed read: `BAD_CELL`→PASS, `READ_TIMEOUT`→FAIL, `BAD_CHECKSUM`→FAIL |
| TC15 | Battery retry then fail | Unit | `battVolts==0`, `battRetryPending==true` → no verdict; pending false → FAIL and halt |
| TC16 | Vision fault always cleared | Unit | Leaving `VIEW_VISION` or a vision-failing SELF TEST emits `CLEAR_VISION_FAULT` |
| TC17 | Gyro drift is wrap-correct | Unit | Baseline 179°, sample −179° → drift +2°, not −358° |
| TC18 | Drift baseline resets on re-entry | Unit | Exit and re-enter `VIEW_GYRO` → drift 0 at entry sample |
| TC19 | Armed force-closes | Unit | `missionArmed == true` → `State::CLOSED` from any state |
| TC20 | Row truncation | Unit | A 40-char label yields a NUL-terminated row ≤ `Console::TEXT_COLS` |
| TC21 | Open latch is one-shot | Unit | `open()` + one tick → `LIST`, cursor 0; repeated `open()` while in `VIEW_LINE` leaves `VIEW_LINE` |
| TC22 | **Vision cannot pass on the boot state** | Unit | Sample fixed at `visionFaultKind==NONE`, `seq` never bumped, `notifyReadDone` never called → core emits `READ_MOSAIC` exactly once and **never** reaches `DONE` on a PASS; at `UI_SELFTEST_CHECK_TIMEOUT_MS` → vision FAIL + `Signal::FAIL`. An implementation judging the boot sample reaches `DONE`/`PASS` and fails this test |
| TC23 | Stale sample never judged | Unit | `seq` frozen with healthy batt/gyro → no verdict; at timeout → FAIL. Bumping `seq` before the deadline → PASS and advance |
| TC24 | Read/judge handshake, happy path | Unit | `READ_MOSAIC` → `notifyReadDone` with `seq+1` + `BAD_CELL` → PASS → `READ_LINE_BOARDS` on entering `Check::LINE` → `notifyReadDone` with `seq+1` + `lineBoardPresent={true,true}` → PASS → `DONE`, `Signal::PASS` |
| TC25 | One request in flight | Unit | Across a full run, no tick carries a second request while a prior one is unacknowledged; each read emitted exactly once per check |
| TC26 | Frame/signal are level-triggered | Unit | `dirty` stays true until `notifyFrameRendered()`; `signal` stays until `notifySignalDone()` — a suppressed repaint is never lost |
| TC27 | Degraded availability truth table | Unit | `boot.drive=false` → `CAL MOTOR`/`CAL LINE`/`GYRO ZERO` non-dispatchable, `BATT`/`SELF TEST`/`EXIT` dispatchable; `boot.batt=false` alone → every read-only entry still dispatchable |
| TC28 | Degraded SELF TEST self-fails | Unit | `boot.yaw=false` → halts at `Check::GYRO`, emits no gyro/vision/line request; `boot.lineArray=false` → `wantsLineData()` stays false |
| TC29 | Sketch compiles | Integration | `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` exits 0 |
| TC30 | Tick never stalls (hardware) | Manual | With a view open and a button held, the `[gate] la=n/m` heartbeat keeps advancing |
| TC31 | LINE VIEW is live (hardware) | Manual | Moving the chassis over a line changes junction type and centroid on screen |
| TC32 | Arm path unaffected (hardware) | Manual | After a failed SELF TEST, one BTN_UP press arms normally |
| TC33 | Closed panel is inert (hardware) | Manual | Mission armed and running: `[gate]` counters and tick cadence match a build with `OperatorPanel.update()` removed from `loop()` |
| TC34 | Console reachable after boot failure | Manual | Unplug the front line board so `LineArray.begin()` fails → long-press still opens `DIAGNOSTICS`, BATT names `LineArray`, BTN_UP does not arm |
| TC35 | `probeBoard()` detects a dead board | Manual | Unplug the front board's I²C cable **after** boot → SELF TEST reports LINE FAIL |

**AC → test map:** AC1→TC1/TC4/TC21; AC2→TC2/TC3; AC3→TC7; AC4→TC6/TC8/TC30; AC5→TC9/TC31; AC6→TC13c; AC7→TC14/TC16; AC8→TC17/TC18; AC9→TC7; AC10→TC27/TC34; AC11→TC10/TC11; AC12→TC12/TC26; AC13→TC13/TC13b; AC13b→TC35; AC14→TC14/TC22/TC24; AC15→TC19/TC32; AC16→TC7/TC8; AC17→TC26/TC30; AC18→TC19; AC19→TC1–TC28; AC20→TC29; AC21→TC33; AC22→TC22/TC23/TC24/TC25; AC23→TC34; AC24→TC27; AC25→TC28.

**Test data:** `ConsoleInput` fixtures built in-test as plain structs — no mocks; the core is pure. A `tick(core, in)` helper advances one frame and returns the `ConsoleFrame`.
**Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|---|---|---|---|---|---|
| T1 | Config constants | — | low | `WRO2026/config/ConfigUi.h`, `ConfigChecks.h`, `Config.h` | Create `ConfigUi.h` with the nine constants in Data Model, Doxygen each. Include from `Config.h` **before** `ConfigChecks.h`. Add the four **Config-only** static asserts. `ConfigChecks.h` must not reference `Console::` and must not include any `src/` header — config is below src and `Config.h` is transitively included by every module. Keep any `//` comment off the `#define` line (the Python config parser's regex breaks on it). |
| T2 | Console PODs | — | low | `WRO2026/src/operator/ConsoleTypes.h` | Create the header exactly as pinned in Contracts. Guard `CONSOLETYPES_H`. `<stdint.h>` only — **no `<Arduino.h>`, no `<MatrixMiniR4.h>`, no `Config.h`**; must compile standalone on the host. Owns all frame geometry and its two self-contained static asserts. Declare `BootReport::clean()` and `isDispatchable()`. Doxygen every enum, struct and field, including the PUSHED/PULLED provenance contract. Satisfies AC1 geometry. |
| T3 | Console core: FSM, list, views | T1, T2 | high | `WRO2026/src/operator/OperatorConsole.h`, `OperatorConsole.cpp` | Implement `OperatorConsoleClass` + `extern OperatorConsole` per the pinned signatures. Owns: item table, scroll-window math, button edge detection, `WAIT_RELEASE` (with a pending-action flag distinguishing await-release from await-completion), `open()`'s one-shot latch, `CLOSED/LIST/VIEW_*` transitions, per-view row formatting, wrap-correct gyro drift against a view-entry baseline, `wantsLineData()`, `isDispatchable()` per AC24 (non-dispatchable rows render `"! "`; confirm is a no-op emitting `ActionRequest::NONE`), and the pull cadence — emits `READ_LINE_BOARDS` in `VIEW_LINE` throttled to `UI_VIEW_SAMPLE_MS` and `READ_MOSAIC` in `VIEW_VISION` throttled to `UI_VISION_SAMPLE_MS`, never a second request while one is in flight. `dirty`/`signal` are level-triggered until acknowledged. **Pure** — no `<Arduino.h>`, no `millis()`, no I/O; time only from `in.nowMs`, and all time comparisons rollover-safe (`(uint32_t)(now - then) >= interval`). Include `config/ConfigUi.h` + `config/ConfigGlobal.h` **directly, never `Config.h`** (the umbrella pulls `<Arduino.h>` and would break AC19). Bounded, always-NUL-terminating row formatter. Satisfies AC1–AC9, AC16, AC19, AC24. |
| T4 | SELF TEST sequencer | T3 | high | `WRO2026/src/operator/OperatorConsole.cpp` | Add the `SELF_TEST` state. Every check runs **latch** (record `sample.seq` and `nowMs` at entry; emit its read action if it has one) → **await** (judge nothing until fresh) → **judge, then dwell `UI_SELFTEST_STEP_MS`** (a minimum for readability, never in place of a verdict). Consult `in.boot` **before** the sample: `boot.yaw==false`→GYRO FAIL without reading; `boot.vision==false`→VISION FAIL without emitting `READ_MOSAIC`; `boot.lineArray==false`→LINE FAIL without emitting `READ_LINE_BOARDS`; `boot.batt==false` still runs the live read (the pack may have recovered). **BATT** — no action; freshness `seq > seqAtEntry`; hold while `battRetryPending`; FAIL when `battVolts < Config::BATT_MIN_VOLTAGE_V` or `== 0.0f`. **GYRO** — no action; freshness `seq > seqAtEntry`; FAIL when `!gyroReadOk \|\| !gyroHealthy`. **VISION** — emits `READ_MOSAIC` once on entry; freshness `notifyReadDone(READ_MOSAIC)` **and** `seq > seqAtEntry`; FAIL on `READ_TIMEOUT`/`BAD_CHECKSUM`, PASS on `BAD_CELL`/`NONE`; `NONE` is a PASS **only** after a completed read; always emit `CLEAR_VISION_FAULT` on exit. **LINE** — emits `READ_LINE_BOARDS` once on entry, holds `wantsLineData()` true until judged; PASS iff every board `< lineBoardCount` has `lineBoardPresent == true` — **never inspect `lineRaw`, never use `lineBoardOk` for the verdict**. **Timeout — fail-closed:** any check not fresh within `UI_SELFTEST_CHECK_TIMEOUT_MS` FAILs and halts, showing `<SUBSYS> NO DATA`. Set `Signal::PASS`/`FAIL`. Same file as T3 — not parallel with it. Satisfies AC11–AC14, AC22, AC25. |
| T5 | `probeBoard()` liveness probe | — | med | `WRO2026/src/perception/WROLineArray.h`, `WROLineArray.cpp` | Add `bool probeBoard(uint8_t boardIdx)`: return false when `boardIdx >= 2 \|\| boardIdx >= Config::LINE_BOARD_COUNT`; select the mux channel via the board's `_pWire`/`_ch` using vendor `ADDR_PCA954X` and `1 << _ch`; then `beginTransmission(MatrixColor_ADDR)` / `return endTransmission() == 0`. Doxygen must state: proves the board answered, inspects no channel value, writes no register, leaves `_lastChannels`/`_lastOnline` untouched, ~1 ms. Use the vendor `#define`s — no magic numbers. `WROLineArray.cpp` is not host-linked (the harness force-includes a mock), so no CMake change; verified by `arduino-cli compile` (T11) + manual TC35. Satisfies AC13b. |
| T6 | Hardware adapter | T2, T3, T5 | med | `WRO2026/src/operator/OperatorPanel.h`, `OperatorPanel.cpp` | Implement `OperatorPanelClass` + `extern OperatorPanel`. `update()` **early-returns at the very top when closed** — the only permitted work while `!OperatorConsole.isOpen()` is reading the two button levels: no sampling, no `readBoard()`/`probeBoard()`, no `YawSource`/`Vision`/`MiniR4.PWR` read, no OLED write, no LED/buzzer drive. When open, follow the pinned adapter tick order exactly. Refresh **only the PUSHED group** on `UI_VIEW_SAMPLE_MS`; **never call `readBoard()`, `probeBoard()` or `readMosaic()` except as execution of a core `ActionRequest`**. Battery retry is a `millis()` deadline — **no `delay()` anywhere in the adapter**. Sample gating: skip `YawSource` when `boot.yaw==false`, line when `boot.lineArray==false`, `Vision` when `boot.vision==false`, leaving sentinels. Never execute a request the core could only have emitted for a non-dispatchable entry — assert-and-ignore rather than trusting the core. Render `ConsoleFrame` when `dirty` and `UI_REPAINT_INTERVAL_MS` elapsed, then `notifyFrameRendered()`. Drive `Signal` on `MiniR4.LED` + `MiniR4.Buzzer` with a **timestamp-driven sequencer** reusing the boot-OK/boot-FAIL patterns — the existing chirp loop uses `delay()` between chirps and must not be copied into an update path; call `notifySignalDone()` when the pattern completes. Push the `EventTrace` strings per Contracts → Naming. Not host-linked. Satisfies AC6, AC7, AC10, AC12, AC21, AC25. |
| T7 | Sketch integration | T6 | high | `WRO2026/WRO2026.ino` | Delete `CalMenuState`, `CalMenuItem`, `CAL_MENU_ITEM_COUNT`, `renderCalMenu()`, `_calMenuState`, `_calMenuSel` (declarations at ~`:62-125`) and the whole menu block in `loop()` (~`:425-524`), **including both blocking release-spins (~`:470-473` and ~`:509-512`)** — derive exact current line numbers yourself, earlier cited ranges proved unreliable. Add `#include "src/operator/OperatorPanel.h"`. In `setup()`: capture the 13 boot health booleans + `battVoltage` into a `Console::BootReport`, call `OperatorPanel.setBootReport(...)` and `OperatorPanel.begin()`. In `cooperativeTick()`: gate becomes `LineFollower.isEnabled() \|\| Drive.isSeekingLine() \|\| OperatorPanel.wantsLineData()` — **preserve the `LINE_ARRAY_TICK_GATE_ENABLED` early return**; suppress only EventTrace's OLED render while open, **keeping its Serial heartbeat alive** (see T10 — this needs a render-suppression API on EventTrace, not skipping `update()`). In `loop()`: **replace the unconditional `if (_setupFailed) return;`** with a narrowed guard — (1) `cooperativeTick()` still runs first, unchanged; (2) the long-press trigger (`OperatorPanel.open()`, using `Config::UI_MENU_LONGPRESS_MS`) and `OperatorPanel.update()` run **regardless of `_setupFailed`**; (3) the BTN_UP arm block is skipped when `_setupFailed \|\| OperatorPanel.isOpen()`, while `_btnUpPrev` is still updated every pass so no stale edge survives; (4) `_setupFailed` is latched for the power cycle and `_missionArmed` can never become true while set; (5) push armed state via `OperatorPanel.setMissionArmed(_missionArmed)` — do not add a second global. Do **not** put the panel in `cooperativeTick()`. Satisfies AC4, AC5, AC15, AC17, AC18, AC23. |
| T8 | Host tests for the core | T3, T4 | low | `WRO2026/test/test_operator_console.cpp` | Implement TC1–TC28 against the real `OperatorConsole.cpp` + `ConsoleTypes.h`, driving `ConsoleInput` structs directly. No mocks — include neither `hal_mocks/` nor `path_mocks/`; that absence is itself the assertion that the core needs no Arduino surface. TC22 is the regression guard for the stale-sample hole and must fail against an implementation that judges the boot state. |
| T9 | CMake target | T8 | low | `WRO2026/test/CMakeLists.txt` | Add one `add_executable(test_operator_console ...)` linking `${SKETCH_DIR}/src/operator/OperatorConsole.cpp` + the test file, plus its `add_test`. **No force-includes, no `MOCK_SOURCES`** — the core is Arduino-free. Sole owner of this file. |
| T10 | EventTrace render split + ADR + docs | T7 | med | `WRO2026/src/util/EventTrace.h/.cpp`, `docs/decisions/076-*.md`, `DECISIONS.md`, `ARCHITECTURE.md` | Add a render-suppression API to `EventTrace` (e.g. `setRenderEnabled(bool)`) so the console can own the OLED **without** stopping the Serial heartbeat or the state polling that `update()` also performs — skipping `update()` wholesale, as the old menu did, loses both. Write ADR-076 (~30–60 lines, standard shape): the `src/operator/` upper-layer domain; the core/adapter split and why (the OLED host-link exclusion); views-are-FSM-states vs actions-are-sanctioned-blocking; the push/pull authority split; liveness-by-ACK not by values. Add one `DECISIONS.md` line and bump the status table. Update `ARCHITECTURE.md` file tree, dependency graph and module table — **and correct `ARCHITECTURE.md:176`**: "Each blocking site spins the registered tick callback" is false for `WROLineArray::calibrate()` and `Drive::calibrateMotorBias()`; split the sentence so only `_drainBlocking()` and `readMosaic()` claim the spin. Satisfies AC17. |
| T11 | Compile + full-suite verification | T7, T9, T10 | low | — | Run `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` — **compile only, never flash**; check `arduino-cli board list` first and force `--build-only` if any tooling is involved. Run the **full** `ctest` suite, not just the new target, plus `uv run --extra dev pytest tools/path_planner/tests/` if any config constant shifted. Report all outputs verbatim. Satisfies AC20. |

Parallel: T1, T2 and T5 are independent and start together. T3 unblocks on T1+T2. T4 waits for T3 (same `.cpp`). T6 waits for T2+T3+T5. T7 waits for T6. T8 waits for T3+T4; T9 waits for T8.

## Notes for Implementer

- **The `WHITE` macro trap.** `MiniR4OLED.h` `#define`s bare `WHITE`/`BLACK`/`INVERSE`. `Config::Color` already guards its enum with `#pragma push_macro("WHITE")`. `OperatorPanel.cpp` includes `<MatrixMiniR4.h>` and calls `setTextColor(WHITE)` — fine. `OperatorConsole.cpp` must never include `<MatrixMiniR4.h>`. Host ctest will **not** catch a violation — only a real `arduino-cli` compile will.
- **Never include `Config.h` from the core.** It pulls `<Arduino.h>`. Include `config/ConfigUi.h` and `config/ConfigGlobal.h` directly — both are `<stdint.h>`-only, and `Vision.h` already sets the direct-sub-header precedent.
- **`readBoard()` and `probeBoard()` are live I²C, not cache reads.** Both run only as `READ_LINE_BOARDS` execution, never per-tick. The OLED shares Wire1.
- **The cached line getters do not self-refresh** when `LINE_ARRAY_TICK_CACHE_ENABLED` is on — that is why the gate must be forced open for the derived fields.
- **`Vision::readMosaic()` is blocking and latching.** Sanctioned blocking site; dispatched from the adapter (which spins the tick callback), never from the core. It overwrites the capture-once latch — acceptable pre-arm because the mission re-captures. Its enum is `VisionClass::Fault`; `Vision` is the extern object.
- **Do not re-introduce a blocking release-spin.** If you are writing `while (MiniR4.BTN_*.getState())`, the answer is a `WAIT_RELEASE` tick.
- **`OperatorPanel.update()` goes in `loop()`, not `cooperativeTick()`.** `cooperativeTick()` is re-entered from inside `Drive::_drainBlocking()` and `Vision::readMosaic()`; putting the panel there would re-enter the console mid-dispatch. The two calibration routines don't re-enter it at all — they block outright — which is the other half of the same reason.
- **Don't claim a tick spin you haven't verified.** `WROLineArray` has no `setTickCallback`. Do not add a wrapper pretending otherwise, and do not assert AC4 over the `CAL LINE`/`CAL MOTOR` dispatch window.
- **All time comparisons must be rollover-safe.** Use `(uint32_t)(now - then) >= interval`, never `now >= then + interval`.
- **`EncoderSource`, `Drive` and `Localization` are not sampled** by any screen. Do not add them "for symmetry".
- **Rollback:** additive apart from T7 and T10. Reverting T7 restores the old menu; the `src/operator/` files, `probeBoard()` and the test target are inert without it.
- **Corrections carried forward from review:** (1) the earlier claim that `readBoard()`'s boolean is an authoritative board-answered signal is **withdrawn** — verified false at `WROLineArray.cpp:659-665`. (2) `ARCHITECTURE.md:176`'s blanket tick-spin claim is false for both calibration routines (T10 fixes it). (3) The boot health count is 12 `begin()` results plus `battOk`, not 13 `begin()` calls.
- **Fallback if bring-up invalidates `probeBoard()`:** if the address ACK proves unreliable through the PCA9548 mux on real hardware, downgrade AC13 to report-only (`LINE: ?`, not a FAIL) and drop LINE from the halt chain. Do **not** substitute an all-zero or two-read-differ heuristic — both false-fail on legitimate darkness, the exact failure the user's constraint exists to prevent.

## Known test gaps (from mutation analysis, recorded 2026-07-20)

The host suite was mutation-tested during execution: 45 mutants against `OperatorConsole.cpp`,
26 killed. The gate-worthy misses were fixed. **These survived and are NOT defended by any
test** — they are live follow-up work, not hypotheticals. Each line is a mutation that changes
real behaviour while the suite stays green.

| Mutation | What ships broken if it regresses |
|---|---|
| `notifyActionDone` accepts reads / a mismatched action; `notifyReadDone` ignores `which` | The whole callback-validation contract (three `@note`s in the header) is untested |
| Pull-cadence throttle removed; view entry no longer disarms it | `UI_VIEW_SAMPLE_MS` / `UI_VISION_SAMPLE_MS` never exercised — **no test completes a read inside a live view**, so the cadence path is dead to the suite |
| `_rowAppend` bound `TEXT_COLS` → `FRAME_COLS` (real overflow) | The bounded formatters' truncation is untested. TC20 exercises `_renderBootFailures`' own limit arithmetic instead, not `_rowAppend` |
| `judgeBattery` drops the `< BATT_MIN_VOLTAGE_V` compare | A low-but-nonzero pack PASSes. TC15 only ever uses `0.0f` |
| `_enterState` no longer releases a read the departed view owned | One dropped `notifyReadDone()` poisons the single read slot for the rest of the session — the failure the largest comment block in the core exists to prevent |
| `_consumeOpenLatch` no longer re-homes the cursor | Re-open after `EXIT` from item 8 lands on `EXIT`. TC21's cursor-reset leg is vacuous (`begin()` already zeroes it) |
| `_close()` no longer clears `_openLatch` | Console re-opens itself the tick after `missionArmed` drops, contradicting `open()`'s `@note` |
| BTN_UP also exits a live view | No test presses BTN_UP inside a view |
| `_enterState(SELF_TEST)` no longer re-homes `_check` | A re-entered SELF TEST resumes mid-sequence |
| SELF TEST **PASS** route drops `_clearVisionPending` | A latched vision fault is handed to the mission after a *passing* self test. TC16 covers only the exit and abort routes |

**Also known, deliberately deferred (documented in `OperatorPanel.h`):** a second verdict of the
same `Signal` kind arriving while the first pattern is still playing is absorbed and acked
without playing — one missing confirmation chirp. The fix is a monotonic signal counter written
by the core on every `_signal` write, which ripples through `ConsoleTypes.h`, the core, the
adapter and the tests.

**`BootReport` flag count is not compile-time tied to `_renderBootFailures`.** The renderer
hand-maintains a 13-entry table; a 14th flag compiles cleanly and that module silently never
appears in the failure list. `sizeof(BootReport) == 20` does NOT catch this (13/14/15/16-bool
variants all measure 20 bytes). The real fix is a `Console::BOOT_FLAG_COUNT` in `ConsoleTypes.h`
that the renderer sizes its arrays from and the test asserts against.
