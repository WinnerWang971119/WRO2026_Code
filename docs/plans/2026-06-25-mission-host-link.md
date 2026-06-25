# Mission host-link (issue #46 Phase 2) Plan

**Goal:** Host-link the real `WRO2026/Mission.cpp` so the Mission queue FSM, its `TASK`-case
dispatch into the real `Tasks.cpp`, and its fault / abort / `declarePosition` recovery paths
are asserted against production code in CI — closing the gap left when Phase 1 deleted the
inline `MissionMirror`.

**Approach:** Add a dedicated `test_mission_dispatch` host target that links real `Mission.cpp`
+ real `Tasks.cpp` against the shared `hal_mocks/` HAL (facade + Drive + EventTrace), a new
`mission_mocks/` dir for the upper-facade surfaces the HAL deliberately doesn't cover
(a **test-controllable** PathTracker plus record/no-op LineFollower / Manipulator / Localization),
and the existing 14-node `fixtures/routes_fixture.cpp` resolved by name. The target mirrors the
`test_mock_hal` / `ctps_dump` isolation recipe (`hal_mocks/` include path first, `mocks/` omitted,
force-includes ordered fixture-routes-first). Real `PathTracker.cpp` / `Drive.cpp` linking stays
mocked here — that is the sibling issue #106.

## Scope

- **In scope:**
  - New `WRO2026/test/mission_mocks/` dir: `PathTracker.h` (controllable), `LineFollower.h`,
    `Manipulator.h`, `Localization.h` (record-and-test-set / no-op, `inline` globals).
  - New `test_mission_dispatch` ctest target linking real `Mission.cpp` + real `Tasks.cpp` +
    `fixtures/routes_fixture.cpp` + `hal_mocks/hal_mocks.cpp`, wired in `WRO2026/test/CMakeLists.txt`.
  - `test_mission_dispatch.cpp` — comprehensive behavioral assertions of the real Mission FSM
    via EventTrace ordered capture + MockSerial capture + MockBTN/Buzzer/LED records + the
    mission_mocks records.
  - Cleanup: refresh the `#46 Phase 2` markers in `test_tasks_dispatch.cpp` + `CMakeLists.txt`;
    update ADR-030's status note; `Closes #46` in the PR.

- **Out of scope:**
  - Host-linking real `PathTracker.cpp` / `Drive.cpp` and deleting their `// MIRROR:` slices —
    that is **#106** (PathTracker/Drive real-link). PathTracker/Drive stay mocked here.
  - Any change to `Mission.cpp` runtime behavior — this is a test-only change. (If the link
    surfaces a genuine Mission bug, that is a separate fix, flagged not silently patched.)
  - A new ADR — Phase 2 is an *application* of ADR-049 (mock HAL) + ADR-030 (link real symbols),
    not a new rule.
  - `MissionScript.cpp` / `buildMission()` — the test authors its own missions via the real
    Mission builder API; the competition mission stays out.

## Decisions

- **Dedicated `test_mission_dispatch` target, not extending `test_tasks_dispatch`** (user choice;
  both approach consultants ENDORSED as *mandatory*, not optional). `test_tasks_dispatch` is on
  the `mocks/` + `task_mocks/` world and uses the *no-op, always-idle* `task_mocks/PathTracker.h`;
  Mission's fault FSM needs a *controllable* PathTracker plus the `hal_mocks/` EventTrace + MiniR4
  (which `mocks/` has no `MiniR4` for). Both cannot coexist in one target without breaking ADR-049's
  ODR mutual-exclusion (`mocks/Drive.h` vs `hal_mocks/Drive.h` share the `DRIVE_H` guard).
- **New `mission_mocks/` dir, `inline` globals, force-included** (user choice) — the
  `ctps_mocks/` / `task_mocks/` precedent; the only ADR-049-legal way to give Mission a
  controllable PathTracker + record/no-op LineFollower/Manipulator/Localization without dragging
  in `PathTracker.cpp` / `Drive.cpp` (#106).
- **`mission_mocks/PathTracker.h` mirrors the REAL 10-value `Fault` enum + exact mnemonics**
  (`DRN`/`OFN`/`WSQ`/`PTU`, `??` default) with an exhaustive `switch` — NOT the stale 7-value stub
  in `task_mocks/` / `ctps_mocks/` (which also wrongly returns `"DTO"` for `DRAIN_TIMEOUT`). Both
  consultants flagged this; verified against `PathTracker.h:239-251` + `PathTracker.cpp:327-342`.
- **`isIdle` and fault are independent knobs** on the PathTracker mock (`mockSetIdle(bool)` orthogonal
  to `mockSetFault`), so the `declarePosition` "busy" guard (`!isIdle() && !hasFault()`) is reachable.
- **Time advanced only via `mockAdvanceMillis` / `mockResetMillis`, never `delay()`** — `hal_mocks.cpp`
  does not define `Serial1` / `mockTickCount`, which the inline `delay()` in `mocks/Arduino.h`
  odr-uses; calling `delay()` would link-fail this target (it links cleanly in `test_tasks_dispatch`,
  which pulls `mocks/mocks.cpp`). The FAULT_BEEPING sub-FSM is millis-indexed, so this matters.
- **Custom include list, NOT `COMMON_INCLUDES`** — `COMMON_INCLUDES` contains `mocks/`, which would
  make `mocks/Drive.h` reachable and break the ODR mutual-exclusion silently (the `DRIVE_H` guard
  drops the second definition with no linker error). Use `${HAL_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR}`
  like `test_mock_hal`.
- **No new ADR** — applies ADR-049 / ADR-030. ADR-030's status note is refreshed (Mission now
  linked; kept "Partially implemented" until #106 lands PathTracker/Drive).
- **Approach consult:** Fable (run as Opus per the skill's Fable-unavailable note) ENDORSED;
  Codex ENDORSED. No veto. Both converged on the Fault-enum fidelity, the `mockAdvanceMillis`
  protocol, and the custom-include-list points (folded in above).

## Acceptance Criteria

- [ ] `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`
      passes on **both** local MSVC and CI g++, with a new ctest named `mission_dispatch` present and green.
- [ ] **No regression:** every previously-passing ctest still passes — especially `test_tasks_dispatch`,
      `mock_hal`, `ctps_dump_selftest`, `manipulator`, `localization_offset`, `yaw_source`.
- [ ] `test_mission_dispatch` links the **real** `Mission.cpp` + **real** `Tasks.cpp` (grep the test for
      no re-introduced `MissionMirror` / `TasksMirror`; the assertions run against the production singletons).
- [ ] `mission_mocks/PathTracker.h` defines the **real 10-value** `Fault` enum (`NONE`..`PIVOT_TRIM_UNDERFLOW`)
      and a `faultMnemonic` whose `switch` is exhaustive and returns the **exact** real strings
      (`OK/NZ/UNR/DRV/BAD/DRN/REV/OFN/WSQ/PTU`, `??` default); `mockSetIdle(bool)` and
      `mockSetFault(kind, fromNode, destNode)` are independent knobs.
- [ ] Comprehensive FSM assertions pass (see Testing Strategy): queue drain order; `start()`
      empty/build-error/normal; GOTO happy/unknown-node/holds-on-not-idle; WAIT/PRINT/BEEP/LED;
      TASK-case dispatch + `T>`/`T+`/`T-` events + real Tasks Serial body capture + unknown-task fault +
      drain-time PathTracker fault (`NAV FAULT OFN` content); FAULT_BEEPING→FAULT_AWAIT_RESET→BTN_DOWN reset; `abort()`; `declarePosition`
      success/busy/unknown/clears-faults.
- [ ] Time is advanced only via `mockAdvanceMillis`/`mockResetMillis` (no `delay()`); the target uses a
      custom include list (`hal_mocks/` first, `mocks/` omitted) and force-includes `routes_fixture.h` first.
- [ ] The three genuinely-unreachable branches (task-body `abort`, execute-time-unknown-task,
      `update()`-during-`inTask`) are documented as intentional gaps with the Phase-1 justification —
      not faked.
- [ ] Cleanup landed: `test_tasks_dispatch.cpp` + `CMakeLists.txt` `#46 Phase 2` markers refreshed;
      ADR-030 status note updated; PR body says `Closes #46`.

## Data Model

Not applicable (host-test target; no runtime data structures). The shapes that matter are the
mock class surfaces, pinned in **Contracts & Interfaces**.

## API Contracts

Not applicable (no network/API surface). The cross-file bindings are the mock class + CMake-helper
signatures in **Contracts & Interfaces**.

## Contracts & Interfaces

Single source of truth for every seam more than one task touches.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/test/mission_mocks/PathTracker.h` | T1 | T3 (drives it), T4 (force-include) |
| `WRO2026/test/mission_mocks/LineFollower.h` | T2 | T3, T4 |
| `WRO2026/test/mission_mocks/Manipulator.h` | T2 | T4 (Tasks.cpp TU) |
| `WRO2026/test/mission_mocks/Localization.h` | T2 | T3, T4 |
| `WRO2026/test/test_mission_dispatch.cpp` | T3 | T4 |
| `WRO2026/test/CMakeLists.txt` | T4 | — |
| `WRO2026/test/test_tasks_dispatch.cpp` (marker refresh only) | T6 | — |
| `docs/decisions/030-harness-links-real-sources-ci-enforced.md` | T6 | — |

### `mission_mocks/PathTracker.h` — the controllable PathTracker seam (T1 owns; T3 consumes)

Self-contained (includes only `<stdint.h>` + real `NavTypes.h`). Guard `PATHTRACKER_H`. `inline PathTrackerClass PathTracker;`.

```cpp
enum class Fault : uint8_t {            // mirror PathTracker.h:239-251 EXACTLY
  NONE=0, NOT_ZEROED=1, UNREACHABLE=2, DRIVE_FAULT=3, BAD_NODE=4, DRAIN_TIMEOUT=5,
  REVERSE_UNAVAILABLE=6, OFF_NODE=7, WALL_NOT_SQUARE=8, PIVOT_TRIM_UNDERFLOW=9 };
struct FaultInfo { Fault kind=Fault::NONE; uint8_t fromNode=0xFF; uint8_t destNode=0xFF; };

static const char* faultMnemonic(Fault);  // exhaustive switch; EXACT strings from PathTracker.cpp:331-342:
                                          // OK/NZ/UNR/DRV/BAD/DRN/REV/OFN/WSQ/PTU, "??" default

// goTo builder (move+copy deleted, [[nodiscard]]): async()/setRouteDirection/setArrivalDirection/setStopAtLine → *this
[[nodiscard]] PathTrackerGoToBuilder goTo(uint8_t goalId);      // Mission's call — record goalId + bump count
[[nodiscard]] PathTrackerGoToBuilder goTo(const char* goalName); // REQUIRED: real Tasks.cpp testOne/test1 call this
                                                                 // overload; omit it and the linked Tasks.cpp TU won't compile

bool isIdle() const;          // returns _mockIdle (default true)
bool hasFault() const;        // returns _mockFault.kind != NONE
FaultInfo getFault() const;   // returns _mockFault
void clearFault();            // bump clearFaultCount; reset _mockFault
void abort();                 // bump abortCount
bool begin(); void update(); void setTickCallback(void(*)());   // no-op lifecycle

// control + capture (test surface)
void mockSetIdle(bool);                                  // INDEPENDENT of fault
void mockSetFault(Fault, uint8_t from=0xFF, uint8_t dest=0xFF);
void mockReset();
// records: lastGoToGoal, goToCallCount, abortCallCount, clearFaultCallCount
```

### `mission_mocks/` record/no-op seams (T2 owns)

- `LineFollower.h` — `LINEFOLLOWER_H`, `inline LineFollowerClass LineFollower;`. No-op except
  **records `setEnabled`**: `setEnabled(bool)` bumps a count + stores `lastEnabled` (so `abort()`'s
  `LineFollower.setEnabled(false)` is assertable). Mirror `task_mocks/LineFollower.h`'s
  `[[nodiscard]]` builder surface; forward-declare `WROLineArrayClass`.
- `Localization.h` — `LOCALIZATION_H`, `inline LocalizationClass Localization;`. **Records
  `declarePosition(uint8_t)`**: `lastDeclaredNode` + count. (`ctps_mocks/Localization.h` is the
  template; extend it to record.)
- `Manipulator.h` — `MANIPULATOR_H`, `inline ManipulatorClass Manipulator;`. Pure no-op; copy
  `task_mocks/Manipulator.h` verbatim (needed only for the `Tasks.cpp` whole-TU compile;
  Mission does not touch it).

### CMake helper (T4) — `add_mission_mock_force_includes(target)`

The target COMPOSES three force-include helpers in this ORDER (do NOT fork the HAL set — reuse the
existing `add_hal_force_includes` so it stays single-sourced; if `hal_mocks/` grows a third
force-included header this target inherits it):

```cmake
#   1. add_fixture_routes_force_include(t)  # routes_fixture.h (guard ROUTES_H) — MUST win FIRST,
#                                           #   before Mission.cpp/FieldMap.h/Localization mock pull routes.h
#   2. add_hal_force_includes(t)            # REUSED as-is: hal_mocks/Drive.h (DRIVE_H) + EventTrace.h (EVENTTRACE_H)
#   3. add_mission_mock_force_includes(t)   # NEW — the 4 mission mocks ONLY (/FI MSVC, "SHELL:-include" g++):
#        ${MISSION_MOCKS_DIR}/PathTracker.h   (guard PATHTRACKER_H)
#        ${MISSION_MOCKS_DIR}/LineFollower.h  (guard LINEFOLLOWER_H)
#        ${MISSION_MOCKS_DIR}/Manipulator.h   (guard MANIPULATOR_H)
#        ${MISSION_MOCKS_DIR}/Localization.h  (guard LOCALIZATION_H)
```
`add_mission_mock_force_includes` must NOT re-list Drive/EventTrace (that's `add_hal_force_includes`'s
job). `<MatrixMiniR4.h>` / `<Arduino.h>` are angle-bracket includes resolved by the include-path order
(`hal_mocks/` first → the HAL facade + the `hal_mocks/Arduino.h` forward to `mocks/Arduino.h`) — NOT
force-included.

### Target wiring (T4)

```cmake
set(MISSION_MOCKS_DIR ${TEST_DIR}/mission_mocks)
add_executable(test_mission_dispatch
    ${TEST_DIR}/test_mission_dispatch.cpp
    ${SKETCH_DIR}/Mission.cpp
    ${SKETCH_DIR}/Tasks.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${HAL_MOCKS_DIR}/hal_mocks.cpp)
# CUSTOM include list — NOT COMMON_INCLUDES (which contains mocks/, breaking ODR mutual-exclusion).
# hal_mocks/ FIRST; mocks/ OMITTED; mission_mocks/ on the path so its quote-included peers resolve.
target_include_directories(test_mission_dispatch PRIVATE
    ${HAL_MOCKS_DIR} ${MISSION_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR})
add_fixture_routes_force_include(test_mission_dispatch)   # FIRST (ROUTES_H)
add_hal_force_includes(test_mission_dispatch)             # REUSED — hal_mocks/Drive.h + EventTrace.h
add_mission_mock_force_includes(test_mission_dispatch)    # the 4 mission mocks only
add_test(NAME mission_dispatch COMMAND test_mission_dispatch)
```

### Global-singleton strategy (the load-bearing seam)

- `hal_mocks/hal_mocks.cpp` (extern + defined): `Drive`, `EventTrace`, `MiniR4`, `Serial`,
  `g_mockMillis`. Linked into this target. Linking it into BOTH `test_mock_hal` and
  `test_mission_dispatch` is **safe** — each `add_executable` is an independent link; duplicate
  symbols only arise within one target.
- `mission_mocks/` facades: `inline` globals (`PathTracker`, `LineFollower`, `Manipulator`,
  `Localization`) — one definition across the target's TUs, no extra `.cpp`. The two sets define
  **disjoint** globals, so extern + inline coexist cleanly.
- **Never link `mocks/mocks.cpp`** into this target (it defines the old-world `Drive` and pulls the
  legacy facade). Never add a `mission_mocks/*.cpp` defining `Drive`/`EventTrace`/`MiniR4`.

### Naming

- Dir: `WRO2026/test/mission_mocks/`. Target: `test_mission_dispatch`. ctest name: `mission_dispatch`.

## Error Handling (build/link failure modes the implementer will hit)

- **`undefined reference to Serial1 / mockTickCount`** → a linked TU odr-used `delay()`. Fix: advance
  time with `mockAdvanceMillis`, not `delay()` (do NOT paper over by adding the symbols to hal_mocks.cpp).
- **Wrong `DriveClass` / duplicate facade / `ROUTES_H` "too many initializers"** → force-include order
  or `mocks/` leaked onto the include path. Fix: fixture-routes `/FI` first; custom include list with
  `mocks/` omitted; verify `hal_mocks/` leads.
- **g++ `cannot specify -o with -c ... multiple files`** → a force-include not wrapped in CMake `SHELL:`.
  Every g++ `-include` must be `"SHELL:-include <abs>"`.
- **MSVC green but g++ red (or vice-versa)** → both toolchains are required gates; build under local MSVC
  (`vcvars64` + `cmake -G Ninja`) AND request the CI g++ run before declaring done. Watch `-Wswitch`
  on a non-exhaustive `faultMnemonic`.
- **A real Mission bug surfaces** → report it; do not silently edit `Mission.cpp` to make a test pass.

## Testing Strategy

**Levels:** Host C++ behavioral unit/integration (ctest). No hardware, no Python. Framework: the repo
`TestAssert.h` `ASSERT_*` macros (exit 0 = pass), same as the sibling targets. Each case calls
`Mission.begin()` + resets the mocks (`mockResetMiniR4()`, `Drive.mockReset()`, `EventTrace.begin()`,
`PathTracker.mockReset()`, `LineFollower`/`Localization` resets, `Tasks.begin()`, `mockResetMillis()`)
at entry, then builds a mission via the real builder API and ticks `Mission.update()`.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `begin()` resets a stale queue | Unit | After enqueue then `begin()`, `isIdle()`==true; an `update()` runs nothing. |
| TC2 | `start()` empty queue | Unit | Serial has "start with empty queue; no-op"; stays IDLE; no `MIS START` event. |
| TC3 | `start()` normal | Unit | Serial "start; steps=N"; `EventTrace` first event `MIS START`; not idle. |
| TC4 | Queue drains in order → done | Integration | print→print mission emits texts in order; on drain Serial "done" + `MIS DONE` event; back to idle. |
| TC5 | GOTO happy path | Integration | ENTER records `PathTracker.goTo(goal)` (count==1); with `mockSetIdle(false)` the step holds (no advance); after `mockSetIdle(true)` → `ARR <name>` event + advance. |
| TC6 | GOTO unknown node | Unit | `goTo("Nowhere")` → Serial "unknown node" + `NAV FAULT BAD` event + FAULT_BEEPING (`Drive.stop(true)` count, `Buzzer.Tone` count, `LED.setColor` red). |
| TC7 | WAIT / PRINT / BEEP / LED | Unit | WAIT advances only after `mockAdvanceMillis(dur)`; PRINT emits event + advances; BEEP records `Buzzer.Tone(freq,dur)`; LED records `LED.setColor` (0-dur advances now, non-zero clears after timeout). |
| TC8 | TASK dispatch (real Tasks) | Integration | `task("demoEcho", uint8_t{7}, uint8_t{42})` (pass TYPED scalars — a braced-init-list `{7,42}` is a non-deduced context for the variadic `task<Args...>` and won't compile) → `T> demoEcho` event before dispatch; MockSerial captures `[Tasks] demoEcho a=7 b=42`; then `T+ demoEcho` event; advances. |
| TC9 | TASK yield | Integration | `task("debugYield")` → real body returns false → `T- debugYield` event + Serial "yielded"; `Tasks.isAborted()`==false; advances. |
| TC10 | TASK unknown at queue time | Unit | `task("nope")` sets build-error (Serial warning); `start()` refuses → `MIS BUILD ERR` event + FAULT_BEEPING; assert NO chassis motion beyond the fault brake (`Drive.mockRecord().driveDistCallCount==0` / `turnDegCallCount==0`; only the FAULT_BEEPING `stop(true)` recorded). |
| TC11 | PathTracker fault during RUNNING (drain-time, OFN content) | Integration | `PathTracker.mockSetFault(OFF_NODE, from, dest)` latched before a tick → the top-of-RUNNING drain check (Mission.cpp:372) fires → `NAV FAULT OFN` event + Serial `FAULT OFN from=<n> to=<n>`; FAULT_BEEPING; the step never advances. (The post-`runTask` check at Mission.cpp:547 is genuinely unreachable with the fixed registry — see Documented unreachable.) |
| TC12 | Fault appears mid-ACTIVE GOTO | Integration | After a GOTO step goes ACTIVE (ENTER fired, awaiting `isIdle`), latch a PathTracker fault → the next `update()` top-of-RUNNING check enters FAULT_BEEPING instead of arriving; the GOTO never reports `ARR`. (Distinct entry sub-state from TC11, same line-372 branch.) |
| TC13 | FAULT_BEEPING schedule | Integration | Advancing the clock by `MISSION_FAULT_BEEP_MS*2` per cycle emits exactly `MISSION_FAULT_BEEPS` `Buzzer.Tone` calls total (the first from `_enterFaultBeeping` on entry, then `MISSION_FAULT_BEEPS-1` more at cycle boundaries) + LED on/off flips; after the last cycle → FAULT_AWAIT_RESET + `PRESS BTN_DOWN` event + LED solid red. |
| TC14 | BTN_DOWN reset | Integration | In FAULT_AWAIT_RESET, `MiniR4.BTN_DOWN.mockSetPressed(true)` rising edge → LED off, `PathTracker.clearFault` (count++), IDLE, Serial "fault cleared" + `MIS RESET` event. |
| TC15 | `abort()` cancels subsystems | Unit | From RUNNING: `Tasks.isAborted()`==true, `PathTracker.abortCallCount`==1, `LineFollower.lastEnabled`==false, `Drive` stop(true) recorded; queue cleared; IDLE. |
| TC16 | `declarePosition` success | Unit | PathTracker idle: `declarePosition("Inter1")` → `Localization.lastDeclaredNode`==id, `Drive.clearFault` called, `DECL Inter1` event, returns true. |
| TC17 | `declarePosition` guards | Unit | `mockSetIdle(false)` (no fault) → false + "PathTracker busy"; `declarePosition("Nowhere")` → false + "unknown node". |
| TC18 | `declarePosition` clears latched faults | Unit | With PathTracker + Drive faults latched: re-anchor clears both (`PathTracker.clearFault` + `Drive.clearFault`), returns true. |

**Documented unreachable (NOT faked):** the **post-`runTask` PathTracker-fault check** (Mission.cpp:547 —
only fires if a fault latches *during* `runTask`; on host `runTask` is fake-blocking and no registry task
latches a PathTracker fault, so the top-of-RUNNING check at line 372 always fires first — TC11/TC12 cover
that reachable branch); task-body `Mission.abort()` bail (`_state != RUNNING` after `runTask`);
execute-time unknown-task (`findTask` null at execute while valid at queue); and `update()`-during-`inTask`
short-circuit — all require a task body the **fixed 8-entry registry** (`demoEcho`, `debugYield`,
`servoTest`, `home`, `testChassis`, `testOne`, `test1`, `test0621`) doesn't provide. The tests invoke only
`demoEcho`/`debugYield`, but ALL 8 compile into the linked `Tasks.cpp` TU, so the mock surface must satisfy
every body's call sites (e.g. `testOne`/`test1`'s `PathTracker.goTo(const char*)` and the servo/belt verbs).
Faking the unreachable branches via a synthetic registry row would assert a fabricated surface (same
justification as `test_tasks_dispatch.cpp`).

**Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure` (locally inside `vcvars64`; CI runs g++ on ubuntu).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | `mission_mocks/PathTracker.h` controllable mock | — | med | `WRO2026/test/mission_mocks/PathTracker.h` | Author per Contracts → PathTracker seam: REAL 10-value `Fault` enum + exhaustive `faultMnemonic` with EXACT strings (verify against `PathTracker.h:239-251` + `PathTracker.cpp:327-342`), `FaultInfo{kind,fromNode,destNode}`, move/copy-deleted `[[nodiscard]]` goTo builder (chain setters → `*this`, record goal at construction — keep the builder self-contained, no facade deref) reachable via BOTH `goTo(uint8_t)` (Mission's call) AND `goTo(const char*)` (real `Tasks.cpp` testOne/test1 use it — omit it and the Tasks TU won't compile), `isIdle`/`hasFault`/`getFault`/`clearFault`/`abort`/`begin`/`update`/`setTickCallback`, independent `mockSetIdle`/`mockSetFault(kind,from,dest)`, capture (`lastGoToGoal`, `goToCallCount`, `abortCallCount`, `clearFaultCallCount`), `mockReset`, `inline PathTrackerClass PathTracker;`. Doxygen + a "Fidelity — where this mock can lie" docblock. Satisfies AC4. |
| T2 | `mission_mocks/` LineFollower + Manipulator + Localization | — | low | `WRO2026/test/mission_mocks/LineFollower.h`, `.../Manipulator.h`, `.../Localization.h` | `LineFollower.h`: copy `task_mocks/LineFollower.h` but make `setEnabled(bool)` record `lastEnabled` + count (+ `mockReset`). `Manipulator.h`: copy `task_mocks/Manipulator.h` verbatim (pure no-op, for the Tasks.cpp TU). `Localization.h`: extend `ctps_mocks/Localization.h` to record `declarePosition(uint8_t)` → `lastDeclaredNode` + count (+ `mockReset`). All `inline` globals, real guards, Doxygen + "Fidelity" docblock. |
| T3 | `test_mission_dispatch.cpp` — comprehensive FSM test | T1, T2 | med | `WRO2026/test/test_mission_dispatch.cpp` | Author TC1–TC18 (Testing Strategy) against the real `Mission` + real `Tasks` singletons. Include real `../Mission.h` + `../Tasks.h`; use `TestAssert.h`. Per-case reset of all mocks + `mockResetMillis`; advance time ONLY via `mockAdvanceMillis` (never `delay()`). Assert via `EventTrace.mockEventAt/mockCount`, `Serial.captured()`, `Drive.mockRecord()`/`getFault`, `MiniR4.Buzzer/LED/BTN_DOWN`, `PathTracker`/`LineFollower`/`Localization` records. Header docblock documents the 3 unreachable branches. Satisfies AC3/AC5/AC7. |
| T4 | Wire `test_mission_dispatch` + `add_mission_mock_force_includes` + refresh CMake marker | T1, T2, T3 | med | `WRO2026/test/CMakeLists.txt` | Add `MISSION_MOCKS_DIR`; author a NEW `add_mission_mock_force_includes(target)` that force-includes ONLY the 4 mission mocks (`/FI` MSVC, `SHELL:-include` g++). Compose force-includes in order: `add_fixture_routes_force_include` (FIRST, ROUTES_H) → REUSE the existing `add_hal_force_includes` (Drive+EventTrace) → `add_mission_mock_force_includes` — do NOT re-list Drive/EventTrace (keep the HAL set single-sourced). `add_executable` + custom include list (`hal_mocks/` first, `mocks/` OMITTED) + link `Mission.cpp`+`Tasks.cpp`+`routes_fixture.cpp`+`hal_mocks.cpp` + `add_test(NAME mission_dispatch ...)`. Refresh the `#46 Phase 2` comment block near `test_tasks_dispatch`. Satisfies AC1/AC6. |
| T5 | Build + run locally (MSVC) + g++ notes | T4 | low | — | `cmake -S WRO2026/test -B build/test -G Ninja` inside `vcvars64`; build + `ctest -C Debug`; confirm `mission_dispatch` + ALL prior targets green (no regression). Document g++-specific concerns (`SHELL:-include`, `-Wswitch`) for the CI gate. Satisfies AC1/AC2. |
| T6 | Cleanup: markers + ADR-030 status | T5 | low | `WRO2026/test/test_tasks_dispatch.cpp`, `docs/decisions/030-harness-links-real-sources-ci-enforced.md` | Refresh the "Out of scope (#46 Phase 2)" notes in `test_tasks_dispatch.cpp` to say Mission is now linked by `test_mission_dispatch`. Update ADR-030's status line: real `Mission.cpp` now linked (keep "Partially implemented" until #106 lands PathTracker/Drive). PR body: `Closes #46`. (Distinct files from T4 — no conflict.) Satisfies AC8. |

T1 and T2 are independent (parallel). T3 waits on T1+T2 (needs the mock surfaces to compile). T4 waits on
T1+T2+T3 (needs the files to exist). T5 waits on T4. T6 waits on T5 (so the markers reflect a verified-green
reality). No two tasks edit the same file.

## Notes for Implementer

- **The whole point is real-link fidelity.** Assert against the production `Mission` + `Tasks` singletons,
  never a copied slice. If a test is awkward to write, fix the mock's observability — do not re-mirror Mission.
- **`mission_mocks/PathTracker.h` is the fidelity-critical file.** The stale 7-value `Fault` enum in
  `task_mocks/`/`ctps_mocks/` (and its wrong `"DTO"` mnemonic) is exactly the drift to avoid — mirror the
  REAL 10-value enum + exact mnemonics. A non-exhaustive `faultMnemonic` switch trips g++ `-Wall`/MSVC `/Wswitch`.
- **Two toolchains, always.** Local MSVC (`cl 19.44` via `vcvars64` + `cmake -G Ninja`) is the fast loop;
  CI g++ is the second required gate. Build under MSVC before declaring done; request the CI g++ run.
- **Never `delay()` in the test** — advance the clock with `mockAdvanceMillis`. `hal_mocks.cpp` omits
  `Serial1`/`mockTickCount`, which the inline `delay()` odr-uses → link failure on this target only.
- **Custom include list, `mocks/` omitted.** Do not reach for `COMMON_INCLUDES`; do not force-include any
  `mocks/` header (the `hal_mocks/Arduino.h` → `mocks/Arduino.h` forward is the only sanctioned bridge).
  `WROLineArray.h`/`Intersection.h` mocks are NOT needed — but ONLY because the mock `LineFollower.h` is
  force-included first (the real `LineFollower.h` would pull them in). That force-include is therefore
  load-bearing, not incidental: drop it and the real `LineFollower.h` leaks `WROLineArray.h`/`Intersection.h`
  (which exist only under the forbidden `mocks/`) and the link breaks.
- **`inline` global trap (future):** mission_mocks use `inline` globals (force-included into one target). If a
  future PR ever defines `Localization`/`PathTracker` in `hal_mocks.cpp`, switch the mission_mocks global to
  `extern` to avoid a multiply-defined symbol — note this where the `inline` global is declared.
- **WHITE macro is a non-issue here** — this target links the EventTrace MOCK (no-op `update()`), not real
  `EventTrace.cpp`, so the `MiniR4OLED.h` `#define WHITE` collision never arises. ADR-049's "add a guarded
  `#define WHITE`" trigger assumes real `EventTrace.cpp` is linked (that is #106-adjacent, not this issue);
  here it is satisfied-by-avoidance, not skipped. Don't add a guard.
- **Fixture nodes resolve by name** (`Inter1`..`Inter4`, `Start`, `final`, `Wall1`, `Spur`, `position1`, ...);
  never by index ([[gotcha-host-tests-link-production-routes]]). Use a bogus name (e.g. `"Nowhere"`) for the
  unknown-node fault.
- **Rollback:** the change is entirely additive (one new dir + one new target + comment/doc refresh). If the
  new target can't go green on both toolchains, it can be excluded from the default build behind a fast-follow
  without touching any existing target.
