# Host-Test Mock HAL Plan (issue #105)

**Goal:** Stand up one shared host-test mock HAL — a unified `MiniR4` facade, a full Drive-builder mock surface, and an `EventTrace` mock — so that later epic-#107 issues (#46 Phase 2 → `Mission.cpp`; #106 → `PathTracker.cpp` + `Drive.cpp`) can host-link those real `.cpp` files and delete the inline `// MIRROR:` slices. This PR builds and self-tests the HAL only; it links no real heavy `.cpp`.

**Approach:** Put the full HAL in its **own** `WRO2026/test/hal_mocks/` dir (mirroring the `ctps_mocks/` precedent), guarded by the real production guards (`MATRIXMINIR4_H`, `DRIVE_H`, `EVENTTRACE_H`) so each header no-ops the production header when force-included first. Add a `add_hal_force_includes()` CMake helper (the 4th copy of the established `/FI`+`SHELL:-include` pattern) and one new `test_mock_hal` target that proves the surface via a signature-conformance TU plus control-surface assertions. Mocks are **thin record-and-test-set** (no multi-tick FSM modeling — settled by an adversarial debate). Finally, perform a green-gated **facade-only consolidation**: collapse `mocks/MatrixMiniR4.h` into a thin forwarding shim over the one `hal_mocks/` facade so there is a single facade definition, reverting any step that can't keep all existing ctests green.

## Scope

- **In scope:**
  - `hal_mocks/MatrixMiniR4.h` — one full `MiniR4` facade mock: `LED` / `Buzzer` / `BTN_UP` / `BTN_DOWN` / `OLED` / `PWR` + a non-template `MockDriveDC` member + `M1`–`M4` + `RC1`–`RC4` + the inline `mmL` (MMLower IMU). Guard = real `MATRIXMINIR4_H`, class named `MatrixMiniR4`, `extern MatrixMiniR4 MiniR4;`.
  - `hal_mocks/Drive.h` — a **copy of the existing near-complete `mocks/Drive.h`** (which already provides the 6 move-only `[[nodiscard]]` builders, `Fault`/`FaultInfo`, `hasFault`/`getFault`/`clearFault`, `isMoveDone`/`isTurnDone`/`isLineFound`/`waitUntilIdle`, `setSpeedSync`/`forwardGyro`, `resetEnc`/`getYaw`/`holdYaw`, `DriveCallRecord` capture, `mockSetIdle`) **extended with the T2 delta**: test-settable fault (incl. `IMU_READ_FAILED`), the full production `Fault`/`FaultInfo` shape (incl. `cmdKind`), `mockSetLineFound`, builder argument capture, and `wallStopTimedOut`. Guard = real `DRIVE_H`. (The issue body's "`mocks/Drive.h` is a LineFollower-only stub" premise is inaccurate — it already exposes the full task-body Drive surface; the real work is the narrow delta above.)
  - `hal_mocks/EventTrace.h` — `EventTraceClass` mock capturing `pushEvent` (ordered ring) + the `MAX_LINE_LEN`/`LINE_HEIGHT_PX` constants Mission sizes buffers from. Guard = real `EVENTTRACE_H`.
  - `hal_mocks/hal_mocks.cpp` — single TU defining the `hal_mocks/` global singletons used by `test_mock_hal` (`MiniR4`, `Drive`, `EventTrace`), unless they are made header-only `inline`.
  - `add_hal_force_includes(target)` CMake helper + the `test_mock_hal` executable + `add_test`.
  - `test_mock_hal.cpp` — (a) a never-run signature-conformance block that writes every real call expression `Drive.cpp` / `Mission.cpp` / `PathTracker.cpp` will make on the HAL; (b) live assertions that the control surface works (set→observe, capture, reset).
  - Green-gated facade-only consolidation: `mocks/MatrixMiniR4.h` → thin forwarding shim over `hal_mocks/MatrixMiniR4.h`; `mocks/mocks.cpp` updated to define the unified `MiniR4` global; all existing ctests must stay green.
  - A new ADR capturing the convention (one shared mock HAL as the host-test trust boundary; thin record-and-test-set fidelity; the dedicated-dir / shadowing rule).
  - Per-header "Fidelity — where this mock can lie" docblock on every new mock (the AC requirement).

- **Out of scope (deferred to dependents):**
  - Linking real `Drive.cpp` / `PathTracker.cpp` / `Mission.cpp` and deleting the `// MIRROR:` slices — that is #106 (PathTracker/Drive) and #46 Phase 2 (Mission). This PR proves the surface compiles/links against a synthetic target only.
  - Consolidating the **behavioral** mocks (`Drive` / `EventTrace` / `PathTracker`) across dirs — explicitly kept per-purpose (recording vs logic-test). Only the MiniR4 facade is consolidated.
  - `ctps_mocks/` facade — `ctps_dump` deliberately excludes `mocks/` and uses a recording facade; it is NOT repointed.
  - The parity harness (#63), the rising-`// MIRROR:`-count CI guard + host-link manifest (#109), the sim↔FSM parity check (#108).
  - Any multi-tick STM32 done-bit / completion modeling inside a mock (debate-rejected).
- **In scope — clarification:** *reading* the three production `.cpp` (`Drive.cpp`/`Mission.cpp`/`PathTracker.cpp`) to grep their HAL call sites for the inventory IS in scope (the surface must match the real call list). Only *linking/compiling* them is out of scope (deferred to #106/#46).

## Decisions

- **Dedicated `hal_mocks/` dir, not grow-in-place** — Both independent approach consultants (Fable + Codex) vetoed growing `mocks/MatrixMiniR4.h` and flipping its guard: that file is on every target's include path, so adding the full facade + switching its guard is a breaking change to all ~27 targets at once and violates the shadowing rule #105 cites. A dedicated dir force-included into one target has a provable blast radius of one new target. The full facade (LED/Buzzer/OLED/DriveDC) is needed by no current target, so it has no business on the shared path now.
- **Real guards from day one** (`MATRIXMINIR4_H` / `DRIVE_H` / `EVENTTRACE_H`) — the `hal_mocks/` facade is the shape #106/#46 will link against; using the production guards means those issues inherit a faithful facade with no second migration, and the guard-shadow trick (force-include first → production header no-ops) works.
- **Thin record-and-test-set fidelity; NO behavioral handshake modeling** — settled by an adversarial `/debate` (Strong No on behavioral modeling, 3.5/10). Modeling the `isPrevTaskDone()` → `_isIdle()` handshake means re-implementing a 6-condition / 4-sub-FSM conjunction whose done-bit is an STM32 coprocessor signal the host can't observe; any invented timing drifts → false green, the exact disease epic #107 treats. Completion/encoder/fault state are plain test-settable fields. Single-call vendor *contracts* (e.g. `resetCounter()` zeroes degrees; error leaves out-params untouched) ARE modeled — that is observation, not logic, and the existing mocks already do it.
- **Non-template `MockDriveDC` member named `DriveDC`** — the vendor's `MiniR4DriveDC<1>` carries `<1>` only to address the lower MCU, which the host never touches; the existing `MockRC`/`MockDC` already drop the template parameter for this exact reason. Follow that precedent.
- **Signature-conformance self-test** — a record-and-test-set mock has little behavior of its own to assert, so the strongest proof #105 can give without linking real `.cpp` is "the surface compiles against the real call list." Conformance TU + light control-surface assertions. **This proof is only as complete as the grep that feeds it**: the conformance block is authored from grepping the three real `.cpp` for every `Drive.`/`MiniR4.`/`EventTrace.` call site (NOT from memory); it asserts the surface matches the grepped real call sites, and #106's real link remains the ultimate check.
- **Facade-only consolidation, green-gated** — the user chose to consolidate the duplicated MiniR4 facade now; `mocks/MatrixMiniR4.h` becomes a forwarding shim over the one `hal_mocks/` facade. The hard gate is "all existing ctests stay green"; any consolidation step that can't meet it reverts to leaving the file untouched + a documented fast-follow. Both consultants flagged in-PR consolidation as the foundational PR's main risk, so it is the LAST, revertible task.
- **`mocks/MatrixMiniR4.h` shim, not COMMON_INCLUDES edit** — the shim uses a source-relative `#include "../hal_mocks/MatrixMiniR4.h"` so no shared CMake include-path change is needed; lowest blast radius.

## Acceptance Criteria

- [ ] `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure` passes on **both** local MSVC and CI g++, with `test_mock_hal` present and green. (Satisfied by T1–T6, T8.)
- [ ] All previously-passing ctests still pass after every task, including after consolidation (T7). No behavioral change to `ctps_dump`, `test_tasks_dispatch`, `test_manipulator`, `test_localization_offset`, `test_yaw_source`. (The hard green gate.)
- [ ] `hal_mocks/MatrixMiniR4.h` exposes, with vendor-faithful signatures, every member the inventory shows `Drive.cpp` + `Mission.cpp` call: `DriveDC.{Move,MoveSync,MoveGyro,MoveDegs,MoveSyncDegs,MoveGyroDegs,MoveTime,MoveSyncTime,MoveGyroTime,brake,begin,setMoveSyncPID,setMoveGyroPID,setTurnGyroPID,isPrevTaskDone,resetCounter}`, `M2/M3.{getDegrees,setReverse,setPPR_RPM}`, `LED.setColor`, `Buzzer.Tone`, `BTN_UP/BTN_DOWN.getState`, `OLED.*`, `PWR.setBattCell`, plus `RC1`–`RC4` + `mmL` carried from the old facade. (T1.)
- [ ] `hal_mocks/Drive.h` exposes, with production-faithful signatures, every Drive method `PathTracker.cpp`/`Mission.cpp` actually call (the inventory is DERIVED FROM the real `WRO2026/Drive.h` public surface via the mandated grep, restricted to members a consumer TU reaches — do NOT add Drive members no consumer calls): the 6 builders + `setSpeed`/`stop`/`isIdle`/`isMoveDone`/`isTurnDone`/`isLineFound`/`hasFault`/`getFault`/`clearFault`/`waitUntilIdle`/`getEncL`/`getEncR`/`resetEnc`/`getYaw`/`wallStopTimedOut`, plus the `Fault`/`FaultInfo` types **in their full production shape** (the 5-value enum incl. `IMU_READ_FAILED` and the 3-field struct incl. `cmdKind: DriveCommand::Kind` — mirror `WRO2026/Drive.h` exactly). Fault / `isLineFound` / `isIdle` are test-settable; builder verbs capture last-commanded args. (T2.)
- [ ] `hal_mocks/EventTrace.h` exposes `begin`/`update`/`pushEvent`/`clear` + `MAX_LINE_LEN`==20 + `LINE_HEIGHT_PX`==8, and captures `pushEvent` strings in order for assertion. (T3.)
- [ ] `test_mock_hal` contains a never-executed signature-conformance block naming every `Drive.`/`MiniR4.`/`EventTrace.` call expression obtained by GREPPING `WRO2026/Drive.cpp`, `Mission.cpp`, and `PathTracker.cpp` (not from memory); the block is only as complete as that grep, so a missing/wrong-signature method on a grepped call is a compile error. The grep-derived call list is captured in the PR so #106 can reuse it as the link checklist. (T5.)
- [ ] `test_mock_hal` control-surface assertions pass: `mockSetFault`→`hasFault`/`getFault`, `mockSetLineFound`→`isLineFound`, `MockDriveDC::mockSetPrevTaskDone`→`isPrevTaskDone`, `mockSetDegrees`/`resetCounter`→`getDegrees`, builder/command capture records last args, `mockReset` clears all state, captured-`pushEvent` order is correct. (T5.)
- [ ] Every new mock header carries a "Fidelity — where this mock can lie vs real hardware/Drive" docblock (faults it never raises on its own, timing it ignores, single-call contracts it does honor). (T1–T3.)
- [ ] A new ADR exists under `docs/decisions/` and is indexed in `DECISIONS.md`. (T9.)
- [ ] Doxygen on every public method, no magic numbers, `clang-format`-clean per the repo `.clang-format`. (All tasks.) **No-dynamic-allocation is NOT required for host test mocks** — the existing harness uses STL heap containers (`mocks/Arduino.h` uses `std::deque`/`std::string`); the firmware no-alloc rule is a runtime-firmware rule, not a host-test rule. Where the spec names a "fixed-size" buffer (EventTrace capture ring) that is for determinism, not memory; other capture state may use STL.

## Data Model

Not applicable (no runtime data structures). The "shapes" that matter are the mock class surfaces, pinned in **Contracts & Interfaces** below.

## API Contracts

Not applicable (no network/API surface). The cross-file binding surface is the mock class/CMake-helper signatures in **Contracts & Interfaces**.

## Contracts & Interfaces

Single source of truth for every seam more than one task touches.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/test/hal_mocks/MatrixMiniR4.h` | T1 | T5 (self-test), T7 (shim), T6 (CMake) |
| `WRO2026/test/hal_mocks/Drive.h` | T2 | T5, T6 |
| `WRO2026/test/hal_mocks/EventTrace.h` | T3 | T5, T6 |
| `WRO2026/test/hal_mocks/Arduino.h` (forward → `../mocks/Arduino.h`) | T4 | T6 |
| `WRO2026/test/hal_mocks/hal_mocks.cpp` | T4 | T6 (linked into `test_mock_hal`) |
| `WRO2026/test/test_mock_hal.cpp` | T5 | T6 |
| `WRO2026/test/CMakeLists.txt` | T6 | T7 |
| `WRO2026/test/mocks/MatrixMiniR4.h` | T7 | — (existing facade consumers) |
| `WRO2026/test/mocks/mocks.cpp` | T7 | — |
| `docs/decisions/0NN-host-mock-hal.md`, `DECISIONS.md` | T9 | — |

### Global-singleton strategy (the load-bearing seam between T1–T4 and T6/T7)

- Each `hal_mocks/` facade/Drive/EventTrace class declares its global `extern` (`extern MatrixMiniR4 MiniR4;` etc.) and `hal_mocks.cpp` (T4) is the single definer — mirroring `mocks/mocks.cpp`. (Alternative: header-only C++17 `inline` globals, dropping `hal_mocks.cpp`; T4 picks one and ALL of T1–T3 must agree. Default = `extern` + `hal_mocks.cpp`, matching the dominant repo pattern; `mmL` stays `inline` as it already is.)
- **`test_mock_hal` links ONLY `hal_mocks.cpp`** (or none if header-only) — never `mocks/mocks.cpp` — so the two facade globals never collide. `hal_mocks.cpp` (T4) must define **every** `<Arduino.h>`-declared global the target odr-uses: `Serial` (`MockSerial`) and `g_mockMillis` for sure (the headers call `millis()`/`Serial`); add `Serial1`/`mockTickCount` only if odr-used — otherwise their `extern` decls are harmless. Determine the set by building; an undefined-reference link error names exactly which to add.
- **ODR mutual-exclusion invariant (non-negotiable):** `mocks/Drive.h` and `hal_mocks/Drive.h` are two *different* `class DriveClass` definitions sharing the `DRIVE_H` guard; likewise the two facades. **No single target may have BOTH `mocks/` and `hal_mocks/` reachable** (on its include path or force-include set). `test_mock_hal` puts `hal_mocks/` first and OMITS `mocks/` (mirror `ctps_dump`); existing targets never see `hal_mocks/` except transitively through the T7 forwarding shim (which is `mocks/MatrixMiniR4.h` only, not `mocks/Drive.h`).

### Facade superset-carry invariant (T1 ↔ T7)

T1 carries `MockDC`/`MockRC`/`MockMMLower` + `namespace MMLower` into `hal_mocks/MatrixMiniR4.h` **type-identically** (same class names, member names, method signatures), so the new `MatrixMiniR4` facade is a strict superset of the old `MockMatrixMiniR4` and the T7 forwarding shim keeps facade consumers green. The T7 ctest green-gate mechanically proves this only for members existing targets consume (`M2`/`M3`/`mmL` and their exercised signatures); this invariant additionally covers the carried-but-unconsumed surface (`M1`/`M4`, `RC1`–`RC4`, untouched overloads) the gate cannot exercise. **T1 must produce a member-by-member diff of `MockMatrixMiniR4` vs the new facade confirming no carried member changed name or signature.**

### Drive Fault/FaultInfo shape (T2) — mirror production exactly

The mock `Fault` enum and `FaultInfo` struct in `hal_mocks/Drive.h` MUST mirror `WRO2026/Drive.h` byte-for-byte (verify against the real header at author time):
```cpp
enum class Fault : uint8_t { NONE=0, DRAIN_TIMEOUT=1, DISPATCH_FAILED=2, LINE_NOT_FOUND=3, IMU_READ_FAILED=4 };
struct FaultInfo { Fault kind = Fault::NONE; uint8_t rc = 0x00; DriveCommand::Kind cmdKind = /*production default*/; };
```
This requires `DriveCommand::Kind` to resolve in the mock surface — mirror or stub the `DriveCommand::Kind` enum in `hal_mocks/Drive.h` (matching the real one's enumerators). #105's self-test need not *exercise* `cmdKind`, but the field+type must be PRESENT so #106's real `PathTracker.cpp` (which reads `getFault().cmdKind`) compiles.

### Drive builder argument-capture seam (T2) — avoid the MSVC C2027 ctor cycle

Builder capture must record at builder **construction**, but the builder classes are defined BEFORE `DriveClass`, so an inline ctor/dtor that dereferences the facade hits MSVC `C2027: use of undefined type 'DriveClass'` (g++ silently accepts → CI-green-but-MSVC-red). Use ONE of: (a) free recording functions the builder ctor calls (the `ctps_mocks/Drive.h` pattern: builders take a forward-declared `DriveClass&` and call free `record*()` writing to a test-inspectable record), or (b) builders hold a `DriveClass*` and the capturing member is defined **out-of-line after** `DriveClass` is complete. Keep builders move-only `[[nodiscard]]`. `TC6` asserts capture **timing** (the verb+arg are recorded at the call site, before any chained setter), not just final state.

### Mock control-surface naming convention (every new mock conforms)

- Reset: `void mockReset()`; per-facade `mockResetMiniR4()` free function inline (mirror existing).
- Setters: `mockSetX(...)` (`mockSetFault`, `mockSetLineFound`, `mockSetIdle`, `mockSetPrevTaskDone`, `mockSetDegrees`).
- Capture: `mockRecord()` returning a `const ...CallRecord&`; counters `xCallCount`, last-arg `lastX`. (Mirror `mocks/Drive.h::DriveCallRecord` + `mocks/MatrixMiniR4.h::MockDC`.)

### MockDriveDC surface (T1) — vendor-faithful signatures

```cpp
class MockDriveDC {
public:
  uint8_t begin(uint8_t l, uint8_t r, bool lRev, bool rRev);   // record, return 0
  bool setMoveSyncPID(float, float, float);                    // record, return true
  bool setMoveGyroPID(float, float, float);
  bool setTurnGyroPID(float, float, float);
  uint8_t Move(int16_t l, int16_t r);                          // record last L/R
  uint8_t MoveSync(int16_t l, int16_t r);
  uint8_t MoveGyro(int16_t power, int16_t target);
  uint8_t MoveDegs(int16_t l, int16_t r, uint16_t deg, bool brake, bool async);
  uint8_t MoveSyncDegs(int16_t l, int16_t r, uint16_t deg, bool brake, bool async);
  uint8_t MoveGyroDegs(int16_t power, int16_t target, uint16_t deg, bool brake, bool async);
  uint8_t MoveTime(int16_t l, int16_t r, float sec, bool brake, bool async);
  uint8_t MoveSyncTime(int16_t l, int16_t r, float sec, bool brake, bool async);
  uint8_t MoveGyroTime(int16_t power, int16_t target, float sec, bool brake, bool async);
  uint8_t brake(bool brakeType);
  uint8_t resetCounter();                                      // zero the encoder backing
  bool isPrevTaskDone();                                       // returns settable mockPrevTaskDone (default true)
  // test surface
  void mockSetPrevTaskDone(bool);
  void mockReset();
  // recorded: lastMoveL/R, lastMoveDegsDeg, *CallCount, ...
};
```
Owner: T1. Consumers: T5 (signature + control assertions). Exact arg types must match `MatrixMiniR4/src/Modules/MiniR4DriveDC.h` — verify against the vendor header at build time.

### CMake helper (T6) — `add_hal_force_includes(target)`

```cmake
function(add_hal_force_includes target)   # MSVC: /FI<abs>;  g++: "SHELL:-include <abs>"
    # force-include hal_mocks/MatrixMiniR4.h? NO — angle-bracket <MatrixMiniR4.h> resolves
    #   via -I hal_mocks/ first (like mocks/ does). Force-include the QUOTE-included
    #   shadows only: hal_mocks/Drive.h (DRIVE_H), hal_mocks/EventTrace.h (EVENTTRACE_H).
endfunction()
```
`test_mock_hal`'s include dirs = `hal_mocks/` FIRST, then `${SKETCH_DIR}`, `${TEST_DIR}` (so `<MatrixMiniR4.h>`/`<Arduino.h>` resolve to the mock dir; `mocks/` is NOT on its path, to avoid pulling the old facade — mirror `ctps_dump`). Owner: T6. Consumer: none (T6 wires the one target).

### Naming

- New dir: `WRO2026/test/hal_mocks/`. New target: `test_mock_hal`. New ctest name: `mock_hal`.
- ADR number: next free integer under `docs/decisions/` (check at author time; never reuse). The plan refers to it as `0NN`.

## Error Handling

- **A consolidation step (T7) breaks a previously-green ctest** → revert that step; leave `mocks/MatrixMiniR4.h` as-is; record the dual-facade as a documented fast-follow in the PR body and the ADR's "Trigger to revisit". The hard green gate wins over maximal dedup.
- **A `MockDriveDC` signature doesn't match the vendor header** → it surfaces as a compile error in T5's conformance TU (or T7); fix the signature against `MiniR4DriveDC.h`, do not loosen the conformance block.
- **A force-include path is de-duplicated by CMake on g++** (the `cannot specify -o with -c ... multiple files` failure) → ensure every `-include` is wrapped in CMake's `SHELL:` prefix (existing gotcha).
- **Local MSVC green but CI g++ red** (e.g. a builder dtor calling a later-defined facade → C2027 the other way, or an `inline` ODR mismatch) → both toolchains are required; reproduce locally with the MSVC build AND request the CI g++ run; do not merge on one toolchain alone.

## Testing Strategy

**Levels:** Host C++ unit/conformance (ctest). No hardware. No Python.

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `test_mock_hal` builds at all (signature-conformance block compiles) | Compile | Every real HAL call expression from Drive/Mission/PathTracker resolves; a removed/renamed mock method fails the build. |
| TC2 | `MockDriveDC` Move/MoveDegs capture | Unit | After `MiniR4.DriveDC.Move(40,-40)`, `lastMoveL==40 && lastMoveR==-40`; `MoveDegs(...)` records deg+brake+async. |
| TC3 | `isPrevTaskDone` test-set | Unit | Default `true`; after `mockSetPrevTaskDone(false)` returns `false`; `mockReset()` restores default. |
| TC4 | `MockDC.resetCounter()` zeroes degrees (single-call contract) | Unit | After `mockSetDegrees(500); resetCounter()`, `getDegrees()==0`. |
| TC5 | Drive fault surface test-set | Unit | `mockSetFault(LINE_NOT_FOUND)` → `hasFault()==true`, `getFault().kind==LINE_NOT_FOUND`, `isLineFound()==false`; `clearFault()` clears all three consistently. |
| TC6 | Drive builder capture timing | Unit | `Drive.stopAtLine(30).setReverse(true).async()` records the verb + expectedCm AT THE CALL SITE (capture happens at builder construction, before chained setters); discarded-temporary call site compiles (move-only `[[nodiscard]]`). |
| TC7 | EventTrace pushEvent ordered capture | Unit | After `pushEvent("A"); pushEvent("B")`, capture log == `["A","B"]`; `clear()` empties it; `MAX_LINE_LEN==20` (19 usable chars + NUL); a 25-char string is truncated to 19. |
| TC8 | `mockReset` clears every facade/Drive/EventTrace field | Unit | All counters/last-args/flags return to defaults. |
| TC9 | Existing suite regression (all prior ctests) | Integration | Every pre-existing ctest still passes after T1–T6 and after T7 — the hard green gate. |
| TC10 | MSVC + g++ parity (run-condition, not a ctest case) | Integration | `test_mock_hal` + full suite green under both toolchains — a CI/local-run gate, not a runnable `add_test` case. |

**Test data:** none (pure in-memory mock state). **Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure` (locally inside `vcvars64`; CI runs g++ on ubuntu).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Build `hal_mocks/MatrixMiniR4.h` full facade | — | med | `WRO2026/test/hal_mocks/MatrixMiniR4.h` | Author the unified facade: class `MatrixMiniR4` (guard `MATRIXMINIR4_H`), `extern MatrixMiniR4 MiniR4;`. Members: `MockLED LED` (`setColor(idx,r,g,b)` + packed overload + `setBrightness`), `MockBuzzer Buzzer` (`Tone`,`NoTone`), `MockBTN<2> BTN_UP`/`MockBTN<1> BTN_DOWN` (`getState()`), `MockOLED OLED` (Adafruit-style no-ops), `MockPower PWR` (`setBattCell`), `MockDriveDC DriveDC` (see Contracts; non-template; record args + settable `isPrevTaskDone`/encoder), `MockDC M1..M4` + `MockRC RC1..RC4` (carry over from the current `mocks/MatrixMiniR4.h` verbatim), inline `MockMMLower mmL` + `namespace MMLower { enum class RESULT }` (carry over verbatim). `begin()`→true. `mockReset()` resets all children; inline `mockResetMiniR4()`. Doxygen on every public method. A "Fidelity — where this mock can lie" docblock. Verify `DriveDC`/`LED`/`Buzzer`/`BTN`/`OLED`/`PWR` signatures against `C:\Users\wangd\documents\WRO2026\MatrixMiniR4\src\Modules\*`. Satisfies AC3. |
| T2 | Extend a copy of `mocks/Drive.h` into `hal_mocks/Drive.h` | — | low-med | `WRO2026/test/hal_mocks/Drive.h` | `hal_mocks/Drive.h` is a **copy of the existing near-complete `mocks/Drive.h`** (guard `DRIVE_H`, `extern DriveClass Drive;`) — which already provides the 6 move-only `[[nodiscard]]` builders, `Fault`/`FaultInfo`, `hasFault`/`getFault`/`clearFault`, `isMoveDone`/`isTurnDone`/`isLineFound`/`waitUntilIdle`, `setSpeedSync`/`forwardGyro`, `resetEnc`/`getYaw`/`holdYaw`, `DriveCallRecord` capture, `mockSetIdle`. Implement ONLY the **delta**: (1) test-settable fault `mockSetFault(Fault)` → `hasFault`/`getFault`, with the full production `Fault` enum (5 values incl. `IMU_READ_FAILED`) + `FaultInfo` (incl. `cmdKind: DriveCommand::Kind`, mirror/stub the `DriveCommand::Kind` enum) per Contracts → Drive Fault/FaultInfo shape; (2) `mockSetLineFound(bool)` → `isLineFound`; (3) builder argument capture per Contracts → builder argument-capture seam (free recording fn OR out-of-line capture after `DriveClass` is complete — NOT an inline ctor touching the facade, the MSVC C2027 trap); (4) `wallStopTimedOut()`. Do NOT add Drive members no consumer calls. Thin — NO multi-tick completion modeling. Doxygen + "Fidelity" docblock. Satisfies AC4. |
| T3 | Build `hal_mocks/EventTrace.h` mock | — | low | `WRO2026/test/hal_mocks/EventTrace.h` | Author a **standalone re-declaration** `EventTraceClass` mock (guard `EVENTTRACE_H`, `extern EventTraceClass EventTrace;`) — do NOT include the real `EventTrace.h` or `Config.h`; it is a fresh class, so it need not match the production private layout. `begin()`→true, `update()` no-op, `clear()` clears capture, `pushEvent(const char*)` appends (truncated to `MAX_LINE_LEN-1` = 19 usable chars, NUL-safe) into a fixed-size ordered capture ring + `mockEvents()`/count accessor + `mockReset()`. `static constexpr uint8_t MAX_LINE_LEN=20; LINE_HEIGHT_PX=8;` (match real). Doxygen + "Fidelity" docblock. Satisfies AC5. |
| T4 | `hal_mocks/Arduino.h` forward + `hal_mocks/hal_mocks.cpp` singleton TU | T1, T2, T3 | low | `WRO2026/test/hal_mocks/Arduino.h`, `WRO2026/test/hal_mocks/hal_mocks.cpp` | `hal_mocks/` must be SELF-CONTAINED (mirroring `ctps_mocks/`, which has its own `Arduino.h`) so `mocks/` never goes on `test_mock_hal`'s include path (ODR mutual-exclusion invariant). The angle-included `<Arduino.h>` is facade-free, so share it via a one-line forward `hal_mocks/Arduino.h` → `#include "../mocks/Arduino.h"` (NOT a copy; it pulls `Serial`/`millis`, no MiniR4). Then `hal_mocks/hal_mocks.cpp` defines the globals the target odr-uses: `MatrixMiniR4 MiniR4; DriveClass Drive; EventTraceClass EventTrace;` + `MockSerial Serial; uint32_t g_mockMillis = 0;` (and `MockHardwareSerial Serial1; uint32_t mockTickCount = 0;` ONLY if the link reports them undefined). Do NOT link `mocks/mocks.cpp`. If T1–T3 chose header-only `inline` globals, this `.cpp` holds only the Arduino globals — decide once, keep T1–T3 consistent (Contracts → Global-singleton strategy). |
| T5 | `test_mock_hal.cpp` conformance + control test | T1, T2, T3 | low | `WRO2026/test/test_mock_hal.cpp` | **First, GREP** `WRO2026/Drive.cpp`, `Mission.cpp`, `PathTracker.cpp` for every `Drive.` / `MiniR4.` / `EventTrace.` call expression — author the conformance block FROM that grep output, not from memory (the block is only as complete as the grep). Capture the grep-derived call list verbatim in the PR body so #106 reuses it as the link checklist. (a) `if (false) { ... }` never-run conformance block writing each grepped call expression (e.g. `MiniR4.DriveDC.Move/MoveSyncDegs/.../isPrevTaskDone`, `MiniR4.M2.getDegrees`, `MiniR4.LED.setColor`, `Buzzer.Tone`, `BTN_DOWN.getState`, `Drive.driveDist(..).async()`, `stopAtLine`, `turnToYaw`, `wallStop`, `isIdle`, `hasFault`, `getFault().cmdKind`, `clearFault`, `isLineFound`, `wallStopTimedOut`, `EventTrace.pushEvent`) — the block compiles (DCE is post-typecheck) so a missing/wrong-signature method is a compile error. (b) live `TestAssert.h` cases TC2–TC8. Use the repo `TestAssert.h` harness. Satisfies AC6/AC7. |
| T6 | Wire `test_mock_hal` target + `add_hal_force_includes` | T4, T5 | med | `WRO2026/test/CMakeLists.txt` | Add `HAL_MOCKS_DIR`; `add_hal_force_includes(target)` (`/FI` MSVC, `SHELL:-include` g++) force-including `hal_mocks/Drive.h` + `hal_mocks/EventTrace.h` ONLY (the angle-bracket `<MatrixMiniR4.h>` resolves via include-path order). `add_executable(test_mock_hal test_mock_hal.cpp hal_mocks/hal_mocks.cpp)`; `target_include_directories(... PRIVATE ${HAL_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR})` — `hal_mocks/` FIRST, `mocks/` OMITTED (mirror `ctps_dump`'s isolation); `add_hal_force_includes(test_mock_hal)`; `add_test(NAME mock_hal COMMAND test_mock_hal)`. Build + run locally (MSVC) and confirm green. Satisfies AC1. |
| T7 | Green-gated facade-only consolidation | T1, T6 | high | `WRO2026/test/mocks/MatrixMiniR4.h`, `WRO2026/test/mocks/mocks.cpp` | First, confirm T1's member-by-member diff (Contracts → superset-carry invariant) shows the new facade is a strict superset. Collapse `mocks/MatrixMiniR4.h` to a thin shim: **REPLACE ITS BODY ENTIRELY** — delete the old `MockMatrixMiniR4` class + `MockDC`/`MockRC`/`MockMMLower` + `namespace MMLower` + the `mmL`/`mockResetMiniR4` defs, leaving ONLY `#include "../hal_mocks/MatrixMiniR4.h"` (keep its own include guard line or drop it — the forwarded header's `MATRIXMINIR4_H` guard governs). Do NOT append the include below the old class (that double-defines). Update `mocks/mocks.cpp` `MockMatrixMiniR4 MiniR4;` → `MatrixMiniR4 MiniR4;`. Run this LAST (before T8). Rebuild + run the FULL ctest suite under MSVC; **every previously-green ctest must stay green** (especially `manipulator`, `localization_offset`, `yaw_source`, `reanchor_at_wall*`, `tasks_dispatch`). If ANY step can't stay green, REVERT this task (it touches only 2 files), leave `mocks/MatrixMiniR4.h` untouched, and record the dual-facade as a documented fast-follow. Do NOT touch `ctps_mocks/`. Satisfies AC2 (the hard gate). |
| T8 | Full-suite parity verification (MSVC + note CI g++) | T6, T7 | low | — | Run the full `cmake/ctest` suite under local MSVC (`vcvars64`), confirm `test_mock_hal` + all prior targets green. Document the g++-specific concerns (`SHELL:-include`, `inline`/ODR) for the CI run; CI g++ is the second required gate. Satisfies AC1/AC2, TC9/TC10. |
| T9 | Write the ADR + index it | T6 | low | `docs/decisions/0NN-host-mock-hal.md`, `DECISIONS.md` | New ADR: "host-test mock HAL as the shared trust boundary." Rule: one shared `hal_mocks/` facade as the trust boundary for host-linking production `.cpp`; mocks are thin record-and-test-set (no multi-tick FSM modeling — observations as test-set inputs); per-target facade mocks live in their own dir (shadowing rule); each mock documents where it can lie. Use the next free ADR number (check `docs/decisions/`; never reuse). Add the index line + status-table bump in `DECISIONS.md`. Satisfies AC8. |

T1, T2, T3 are independent (parallel). T4 and T5 wait on T1–T3. T6 waits on T4+T5. T7 waits on T1+T6. T9 waits on T6. T8 waits on T6+T7. T7 is the only `high` task (it edits shared files under the green gate); everything else is additive.

## Notes for Implementer

- **The shadowing rule is the whole point.** `hal_mocks/` headers must be reachable ONLY by `test_mock_hal` (force-include for the quote-included `Drive.h`/`EventTrace.h`; include-path-first for the angle-included `<MatrixMiniR4.h>`). Never add `hal_mocks/` to `COMMON_INCLUDES`. Never add a `hal_mocks/` header to `mocks/`.
- **Two toolchains, always.** Local MSVC (`cl 19.44`, via `vcvars64` + `cmake -G Ninja`) is the fast loop; CI g++ is the second gate. Known divergences: g++ needs `SHELL:-include` (else CMake de-dups the flags); MSVC catches builder-dtor-before-facade `C2027` that g++ silently accepts; watch `inline`-global ODR. Build under MSVC before declaring done.
- **Carry over `MockDC`/`MockRC`/`MockMMLower`/`namespace MMLower` verbatim** from the current `mocks/MatrixMiniR4.h` into `hal_mocks/MatrixMiniR4.h` so the consolidated facade is a strict superset — that is what lets T7's shim keep `manipulator`/`localization_offset`/`yaw_source` green.
- **Verify vendor signatures, don't guess.** `MockDriveDC` / `MockLED` / `MockBuzzer` / `MockBTN` / `MockOLED` / `MockPower` arg types must match `C:\Users\wangd\documents\WRO2026\MatrixMiniR4\src\Modules\*` (e.g. `MiniR4DriveDC.h`, `MiniR4LED.h`). A wrong arg type passes the self-test compile but breaks #106's real link — defeating the HAL's purpose.
- **Thin, not dumb.** Model single-call vendor contracts the SUT depends on (`resetCounter()` zeroes degrees; `mmL.GetIMUEuler` leaves out-params untouched on error). Do NOT model multi-tick completion. If you feel tempted to make `isPrevTaskDone()` "eventually return true on its own," stop — that's the rejected behavioral path.
- **Self-test honesty.** The conformance block proves the *surface*; the control assertions prove the *test API*. Neither re-derives production logic. Do not assert "PathTracker would advance" here — that's #106.
- **Rollback plan for T7:** the consolidation is the only risky edit. It is last and self-contained (two files). If it can't go green, revert those two files; the rest of the PR (the new `hal_mocks/` dir + `test_mock_hal`) stands alone and still fully delivers #105's goal. The dual-facade dedup becomes a one-line fast-follow.
