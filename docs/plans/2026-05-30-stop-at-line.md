# Drive.stopAtLine() + PathTracker re-anchor Plan

**Goal:** Add a reusable `Drive.stopAtLine()` motion primitive that drives the chassis straight toward an expected perpendicular crossing line, detects it near the expected endpoint, and stops on it with an active brake — then wire `PathTracker` to use it on flagged blind hops so arrivals re-anchor on the physical line instead of dead-reckoning (issue #27: "better localization").

**Approach:** A new RAII fluent builder `Drive.stopAtLine(expectedCm)` (matching the `driveDist` family) drives at cruise, **arms** a `T_OR_CROSS` line detector only once the chassis is within a Config window of the expected endpoint (so it cannot false-trigger on an earlier line), rolls a configurable overshoot past the detected line, then active-brakes. It is mode-selectable (`.setMode`, default `SYNC`+`tShape`), forward/reverse, sync-blocking by default with `.async()`. `Drive` gains a **read-only** dependency on the `LineArray` sensor singleton (recorded in a new ADR). `PathTracker` triggers it via a new per-edge `BlindMode::STOP_AT_LINE` override plus a tri-state `goTo().setStopAtLine(AUTO|ON|OFF)` call-site flag (call wins over the map); on success it advances `Localization` via `onIntersection()` (true re-anchor). The Python pipeline + sim mirror the new mode for parity.

## Source request (verbatim)

> Issue #27 — *Drive stopAtLine()*: "# Idea — Add a stopAtLine() function for the drive. It should stop when detected line."
> Comment: "So the PathPlanner system should also use this to navigate and also to provide better locolizationing system."

## Scope

- **In scope:**
  - New `Drive.stopAtLine(float expectedCm)` fluent builder + `StopAtLineBuilder` + a parallel state machine in `Drive::update()`, plus a new Drive fault `LINE_NOT_FOUND` and a `Drive.isLineFound()` query.
  - `Drive` reads `LineArray.getJunctionType()` (read-only) to detect the crossing; new `#include "WROLineArray.h"` in Drive. New ADR-027 records this Drive→perception-leaf dependency.
  - New `BlindMode::STOP_AT_LINE` across the Python pipeline (`pipeline_config.py`, `header_writer.py`, `compiler.py`, `validator.py`) and the generated `routes.h`/`routes.cpp`; authorable per-edge in `field_data`.
  - Tri-state `StopAtLine{AUTO,ON,OFF}` enum (NavTypes.h) on `PathTrackerGoToBuilder.setStopAtLine(...)` and `Mission.setStopAtLine(...)`; call-site value wins over the edge override (OFF can force-disable).
  - `PathTracker` `STOP_AT_LINE` FSM state: dispatch `Drive.stopAtLine(edge.distance_cm).async()`, re-anchor via `Localization.onIntersection()` on success, `Fault::DRIVE_FAULT` on no-line.
  - Reverse support via the rear board (gated on `Config::LINE_BOARD_COUNT == 2`), **coded but marked untested-on-hardware** until the rear board is wired.
  - New `Config` constants (arm window, max-search margin, detect-debounce ticks).
  - Python sim parity (`sim/runtime.py`): a `STOP_AT_LINE` blind edge advances localization via `on_intersection`.
  - Python pytest coverage + C++ host-harness coverage (PathTracker behavior via the Drive mock).
  - Docs: ADR-027, DECISIONS.md index, ARCHITECTURE.md dependency-graph edit, Roadmap note.

- **Out of scope:**
  - `STOP_AT_LINE` on `LINE` or `PROJECTION` edges — BLIND edges only (a projection/line segment that ends at a crossing is already a LINE edge; see Decisions).
  - Opportunistic mid-blind re-anchoring (watching for unexpected lines outside the arm window) — explicitly rejected for being fuzzy.
  - Adding a new `EdgeType` — we reuse the existing `BlindMode` override field; no schema-level edge-type change.
  - On-hardware tuning of the arm window / detection thresholds — that is a Phase 3 bench task, not this PR (defaults shipped, tuned later).
  - Camera, Manipulator, or any unrelated module.

## Decisions

- **Verb lives on `Drive`** (not LineFollower/PathTracker) — **the user chose this explicitly after being shown the tradeoff.** It matches the issue title and the `driveDist` fluent-builder family; `WROLineArray` is a leaf sensor so `Drive → WROLineArray` stays acyclic. The strongest rejected alternative was `LineFollower` (it already holds the `WROLineArray` pointer, a distance gate, brake-on-gate, and REAR-board selection, so it would add *zero* new dependency edges) — rejected because the seek is a `Drive`-mode (SYNC/GYRO bounded-motion) concept LineFollower would have to proxy, and the issue + the user want the verb on `Drive`. Trade accepted: a new cross-stack read dependency, justified in ADR-027.
- **`stopAtLine` is a CONTINUOUS bespoke move, NOT the bounded `tShape` profile machine.** It runs its own state machine (`_updateStopAtLine`) that commands continuous motion each tick (`Move`/`MoveSync`/`MoveGyro` per `.setMode`) and stops on an *external event* (line detected) or a max-distance bound. The existing `_updateProfile` (which needs a known total distance to plan a ramp-down) does **not** run for a `stopAtLine` move. `tShape` here means an *optional bespoke ramp-up* (0→cruise) over the lead-in; the seek phase cruises flat (the stop point is unknown until detection); the only ramp-down is the known-length overshoot roll. The two machines never co-own the wheels — dispatching a `stopAtLine` cancels any in-flight profile, same as any other bounded move.
- **Detect on `getJunctionType() == T_OR_CROSS`** (full bar crossing), read **directly** by the seeker with its own rising-edge + N-tick debounce. It does **not** consume `Intersection.isAtIntersection()` (read-and-clear, reserved for PathTracker's nav consumer) — avoiding cross-consumer contention.
- **Arming window** — drive blind until `traveled ≥ expectedCm − armWithin`, *then* arm detection. Prevents stopping on a line crossed earlier en route. Default `armWithin = Config::STOP_AT_LINE_ARM_WINDOW_CM`, overridable per call. **Tuning constraint (load-bearing):** the window guards against *early* (already-passed) lines, but it only fires on the true line if the encoder estimate of `traveled` is within `armWithin` of reality. If dead-reckoning *under-travels* by more than `armWithin`, the chassis crosses the real line while still disarmed and misses it (→ runs to max → fault). So `armWithin` MUST be sized larger than the worst-case dead-reckoning under-travel error over the hop — this is exactly the drift the feature exists to correct, so the window must be generous, not tight. Verified on hardware in B3.7.
- **Fault on no-line** — if `traveled ≥ expectedCm + Config::STOP_AT_LINE_MAX_MARGIN_CM` without a detection, Drive fail-fast faults `LINE_NOT_FOUND` (stop + clear queue), surfaced to PathTracker as `DRIVE_FAULT`. No silent dead-reckon fallback.
- **Stop point** — on detection, roll forward a configurable overshoot (default `Config::LINE_SENSOR_AXLE_OFFSET_CM`) so the wheel axle lands on the line, smooth-ramp to `PWM_MIN_MAGNITUDE` then active-brake. There is no existing reusable smooth-stop helper — T4 writes this ramp (the bounded `_updateProfile` RAMP_DOWN is structurally similar but not callable standalone).
- **Default motion `SYNC`** (ramp-up via the bespoke seek ramp), mode-selectable via `.setMode`. Mirrors the project-wide `DriveMode::SYNC` default (ADR-017).
- **Map trigger = `BlindMode::STOP_AT_LINE`** on BLIND edges; **call trigger = tri-state `StopAtLine` flag**; the call-site ON/OFF wins over the edge override; AUTO defers to the edge (mirrors `arrival_direction` precedence).
- **Re-anchor via `onIntersection()`** on success (the "better localization"); plain blind edges keep `onDeadReckonedAdvance()`.

## Data Model

### NavTypes.h — new tri-state flag

```cpp
enum class StopAtLine : uint8_t
{
    AUTO = 0,    ///< Defer to the per-edge BlindMode override.
    ON   = 1,    ///< Force stop-at-line on this hop's BLIND edge (overrides edge).
    OFF  = 2,    ///< Force-disable; traverse the BLIND edge as a normal dead-reckoned hop.
};
```

### pipeline_config.py / routes.h — BlindMode gains a value

```python
class BlindMode(IntEnum):
    DEFAULT     = 0
    SYNC_TSHAPE = 1
    GYRO_TSHAPE = 2
    RAW_FLAT    = 3
    STOP_AT_LINE = 4   # drive blind, arm near endpoint, stop on the crossing line, re-anchor
```
```cpp
// generated routes.h (emitted by header_writer.py):
enum class BlindMode : uint8_t { DEFAULT = 0, SYNC_TSHAPE = 1, GYRO_TSHAPE = 2, RAW_FLAT = 3, STOP_AT_LINE = 4 };
```

### Drive.h — new command kind, builder, state, fault

```cpp
// DriveCommand::Kind gains:
STOP_AT_LINE,   ///< Drive-until-line move (mode-driven; stops on T_OR_CROSS near endpoint).

// DriveClass::Fault gains:
LINE_NOT_FOUND = 3,   ///< stopAtLine searched the full max distance with no crossing detected.

// New private state machine:
enum class StopAtLineState : uint8_t { IDLE, SEEK, ARMED, OVERSHOOT, DONE };

// New STOP_AT_LINE command fields (added to DriveCommand or a dedicated sub-struct):
float   expectedCm;   // nominal distance to the line
float   armWithin;    // arm detection within this many cm of expectedCm
float   maxCm;        // fault if no line by here (default expectedCm + STOP_AT_LINE_MAX_MARGIN_CM)
float   overshoot;    // post-detect roll (default LINE_SENSOR_AXLE_OFFSET_CM)
bool    reverse;      // rear-board, backward motion
// mode/power/heading/brake reuse the existing DriveCommand fields, BUT a
// STOP_AT_LINE command is dispatched into the new _updateStopAtLine() machine,
// NOT _updateProfile() — it is a continuous event-terminated move (see Decisions).
```

### Config.h — new constants

```cpp
constexpr float    STOP_AT_LINE_ARM_WINDOW_CM  = 10.0f;  // arm detection within this of expected endpoint
constexpr float    STOP_AT_LINE_MAX_MARGIN_CM  = 15.0f;  // max search = expected + margin → fault
constexpr uint8_t  STOP_AT_LINE_DETECT_TICKS   = 2;      // consecutive T_OR_CROSS ticks to confirm
// overshoot default reuses Config::LINE_SENSOR_AXLE_OFFSET_CM (= 5.0f)
```

## API Contracts

### Drive (firmware)

```cpp
[[nodiscard]] StopAtLineBuilder Drive.stopAtLine(float expectedCm);

// StopAtLineBuilder knobs (all return *this), RAII auto-execute on destruction:
.setMode(DriveMode m)            // default SYNC
.setPower(int16_t p) / .setPreset(Config::SpeedPreset)   // default SPEED_NORMAL
.setHeading(int16_t yaw)         // GYRO target (default held yaw)
.setArmWithin(float cm)          // default STOP_AT_LINE_ARM_WINDOW_CM
.setMaxDistance(float cm)        // default expectedCm + STOP_AT_LINE_MAX_MARGIN_CM
.setOvershoot(float cm)          // default LINE_SENSOR_AXLE_OFFSET_CM
.setReverse(bool)                // default false (rear board; needs LINE_BOARD_COUNT==2)
.setBrake(bool)                  // default true
.async() / .setBlocking(bool)    // default sync-blocking

bool Drive.isLineFound() const;  // latched result of the last stopAtLine (false if it faulted/maxed)
// On no-line: Drive.hasFault() == true, getFault().kind == Fault::LINE_NOT_FOUND.
```

Usage:
```cpp
Drive.stopAtLine(40);                               // SYNC, arm at 30cm, axle-overshoot, sync-block
Drive.stopAtLine(40).setMode(DriveMode::GYRO).async();   // heading-locked, pollable
Drive.stopAtLine(40).setReverse(true);              // back into the line via rear board
```

### PathTracker / Mission (firmware)

```cpp
PathTracker.goTo("X").setStopAtLine(StopAtLine::ON);   // force final-resolved blind hops to stop-at-line
Mission.goTo("X").setStopAtLine(StopAtLine::OFF);      // force-disable even if the edge says STOP_AT_LINE
```

### Python pipeline

```python
# field_data EDGES per-edge override:
{"to": "Node", "blind_mode": "STOP_AT_LINE"}   # validator accepts; compiler emits the action below

# compiler.py forward action (FRONT):
(ACT_STOP_AT_LINE, distance_cm)
# reverse (REAR) reuses the existing blind-reverse tuple carrying the mode:
(ACT_BLIND_REVERSE, target_node, distance_cm, BlindMode.STOP_AT_LINE)
```

## Error Handling

- **No line within max distance:** `Drive` active-brakes, clears its queue, latches `Fault::LINE_NOT_FOUND`. `PathTracker`'s `STOP_AT_LINE` state observes `Drive.hasFault()` and enters `Fault::DRIVE_FAULT`; `Mission` runs its fault beep-and-flash. `isLineFound()` returns `false`.
- **Under-travel drift larger than the arm window:** if the encoder under-estimates distance by more than `armWithin`, the chassis crosses the real line while still disarmed, never detects, and runs to max → `LINE_NOT_FOUND`. This is the failure mode the arm window must be tuned against (see Decisions / B3.7): size `armWithin` to exceed worst-case under-travel, not to be tight.
- **Reverse requested with `LINE_BOARD_COUNT == 1`:** the rear board does not exist, so `setReverseActive(true)` is a no-op and the front board would read while backing up. Drive rejects this at dispatch (logs a Serial warning, faults `LINE_NOT_FOUND` immediately) rather than reading a wrong-facing board; PathTracker's existing reverse guard (`REVERSE_UNAVAILABLE`, ADR-024) already prevents emitting a reverse hop on a 1-board chassis, so the map path cannot reach this — the guard is defense-in-depth for the bare Drive verb.
- **`expectedCm − armWithin < 0` (or `armWithin ≥ expectedCm`):** clamp the arm threshold to 0 — detection is armed from the first tick. Documented as the small-hop / "already near the line" case.
- **Line already under the sensor at start:** ignored — detection only arms inside the window, so an inbound line at the origin cannot trigger a premature stop (this is the whole point of the arm window).
- **I2C read failure during seek:** `getJunctionType()` returns `0` (NONE) on failure, so a dropped read simply does not trigger; the move continues until detection or max distance. No special handling.
- **New move issued while a stopAtLine is in flight:** same semantics as the existing profile machines — the new command cancels the in-flight stopAtLine (its motors keep running until the new command's first tick). Callers sequence via the queue / `isIdle()`.

## Testing Strategy

**Levels:** Unit (Python pytest, host-compiled C++ harness), plus Phase-3 on-hardware bench verification (manual, not CI).

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `compiler`: BLIND edge with `blind_mode=STOP_AT_LINE`, FRONT | Unit (py) | Emits `(ACT_STOP_AT_LINE, distance_cm)` |
| TC2 | `compiler`: same edge, REAR direction | Unit (py) | Emits `(ACT_BLIND_REVERSE, v, distance_cm, BlindMode.STOP_AT_LINE)` |
| TC3 | `header_writer`: generated `routes.h` BlindMode enum | Unit (py) | Contains `STOP_AT_LINE = 4`; golden routes regenerate byte-stable |
| TC4 | `validator`: `field_data` edge override `"blind_mode": "STOP_AT_LINE"` | Unit (py) | Accepted; an unknown mode string still rejected |
| TC5 | `runtime_sim`: walk a path over a `STOP_AT_LINE` blind edge | Unit (py) | Localization advances via `on_intersection` (not `on_dead_reckoned_advance`); goal position matches the LINE-arrival cross-check |
| TC6 | `runtime_sim`: `STOP_AT_LINE` reverse blind edge | Unit (py) | Advances via `on_intersection`; chassis ends at node |
| TC7 | Mirrored STOP_AT_LINE slice: line-found path re-anchors via the mirrored `onIntersection` call (production `PathTracker.cpp` dispatch NOT executed) | Mirror-slice (C++) | Mirror advances via `onIntersection(nextHop)` → `PLAN_NEXT_HOP` |
| TC8 | Mirrored STOP_AT_LINE slice: Drive reports `LINE_NOT_FOUND` | Mirror-slice (C++) | Mirror enters `Fault::DRIVE_FAULT`; records stop + queue-clear |
| TC9 | Mirrored precedence: `setStopAtLine(OFF)` on a `STOP_AT_LINE` edge | Mirror-slice (C++) | Resolves to a normal blind move (no stop-at-line dispatch) |
| TC10 | Mirrored precedence: `setStopAtLine(ON)` on a plain (`DEFAULT`) blind edge | Mirror-slice (C++) | Resolves to a stop-at-line dispatch |
| TC11 | Arm-window + debounce math — pure free functions in `WROMath.h`: `stopAtLineArmed(traveledCm, expectedCm, armWithin)` (bool, clamp `expectedCm − armWithin ≥ 0`) and the detection debounce counter | Unit (C++, host, **committed** — compiled into the existing LineFollower host target) | Detection ignored before the window; honored inside it; `expectedCm < armWithin` arms immediately; debounce requires N consecutive reads |
| TC12 | compiler↔sim FRONT-blind parity after STOP_AT_LINE coupling | Unit (py, `test_property_random_walks`) | Compiler action stream and sim action stream stay byte-identical for STOP_AT_LINE edges |

**Test data:** Python uses the existing toy field + a fixture edge marked `STOP_AT_LINE` (add to `tests/fixtures`). C++ adds a new host-compile target that **inline-mirrors** the STOP_AT_LINE FSM slice (re-anchor / DRIVE_FAULT / flag-precedence decision), following the `test_path_tracker_passthrough.cpp` precedent. No production `PathTracker.cpp` is linked and no `Drive.stopAtLine` mock is used; the mirrored slice is annotated `// MIRROR: PathTracker.cpp:<line>` for drift detection at review time. The pure arm/debounce free functions (TC11) compile into the existing LineFollower host target.
**Run command:** `uv run pytest tools/path_planner/tests/` (Python); the C++ harness runs in CI via `cmake`/`ctest` (no host C++ compiler locally — verified in CI only).

> **Test-coverage limitation (state honestly):** AC2's arm-window decision and the detection debounce are verified host-side by **TC11 against pure functions extracted from the stateful machine** (a committed host test, not a review item). What remains uncovered by host tests: (1) the production `Drive._updateStopAtLine` *stateful* behavior — physical overshoot-then-brake (PWM ramp + encoder roll) of AC3 — and (2) the production `PathTracker` STOP_AT_LINE dispatch/re-anchor path. **TC7–TC10 assert against an inline-mirrored copy of the PathTracker slice that can silently diverge from production** (the harness does not link `PathTracker.cpp`). Both are covered only by code review plus the Phase-3 hardware bench (**B3.7**). **AC10's "CI C++ harness passes" does NOT prove the production `Localization.onIntersection(_nextHop)` call is correct.** Adding a real PathTracker/Drive SUT build target is a larger harness change tracked as a follow-up (matching the existing project rationale for not linking production `.cpp`s).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Config constants | — | low | `WRO2026/Config.h` | Add `STOP_AT_LINE_ARM_WINDOW_CM=10.0f`, `STOP_AT_LINE_MAX_MARGIN_CM=15.0f`, `STOP_AT_LINE_DETECT_TICKS=2` near `LINE_SENSOR_AXLE_OFFSET_CM`. No magic numbers elsewhere. Satisfies AC2/AC3 inputs. |
| T2 | StopAtLine enum | — | low | `WRO2026/NavTypes.h` | Add `enum class StopAtLine : uint8_t { AUTO=0, ON=1, OFF=2 };` with Doxygen. Satisfies AC6. |
| T3 | Drive.h surface | T1 | med | `WRO2026/Drive.h` | Declare `stopAtLine(float)` verb + `[[nodiscard]] StopAtLineBuilder` (header-only RAII, copy/move-deleted like `DriveDistBuilder`, knobs per API Contracts); add `DriveCommand::Kind::STOP_AT_LINE` + fields; `Fault::LINE_NOT_FOUND=3`; `StopAtLineState` enum + members; `bool isLineFound() const;`; `void _updateStopAtLine();` decl. Friend the builder. **Do NOT add `#include "WROLineArray.h"` here** — the header needs none of WROLineArray's definition (only the .cpp reads the sensor), and including it in Drive.h would pull `MatrixMiniR4`/`MatrixColor` into every TU that includes Drive.h. Satisfies AC1. |
| T4 | Drive.cpp state machine + pure helpers | T3 | high | `WRO2026/Drive.cpp`, `WRO2026/WROMath.h` | Add `#include "WROLineArray.h"` in **Drive.cpp** (not the header). Implement builder enqueue; `_dispatchNext` STOP_AT_LINE case (disable LineFollower brake=false, capture start enc, set `_moveDone=false`, snapshot GYRO heading, `setReverseActive(reverse)`, reject reverse+1-board); `_updateStopAtLine()` = a CONTINUOUS event-terminated machine SEEK→ARMED→OVERSHOOT→DONE that commands continuous motion each tick per `.setMode` (it does NOT run `_updateProfile`); seek cruises flat; OVERSHOOT writes the new smooth ramp-to-`PWM_MIN_MAGNITUDE`-then-brake; T_OR_CROSS read via `LineArray.getJunctionType()` with `STOP_AT_LINE_DETECT_TICKS` debounce; `LINE_NOT_FOUND` fault on max distance; restore `setReverseActive(false)` on done; hook `_updateStopAtLine()` into `update()` parallel to `_updateProfile`; include it in `_isIdle()`. **Extract the pure decision logic into `WROMath.h` free functions** — `bool stopAtLineArmed(float traveledCm, float expectedCm, float armWithin)` (clamp `expectedCm−armWithin ≥ 0`) + the debounce counter step — so it is host-testable (TC11). Satisfies AC1/AC2/AC3/AC8. |
| T5 | pipeline BlindMode value | — | low | `tools/path_planner/pipeline_config.py` | Add `BlindMode.STOP_AT_LINE = 4` + docstring. Satisfies AC4. |
| T6 | header_writer enum emit | T5 | low | `tools/path_planner/header_writer.py` | Append `STOP_AT_LINE = 4` to the emitted `BlindMode` enum string. Satisfies AC4. |
| T7 | compiler action (FRONT now mode-sensitive) | T5 | med | `tools/path_planner/compiler.py` | Add `ACT_STOP_AT_LINE = "stop_at_line"`; FRONT BLIND+STOP_AT_LINE → `(ACT_STOP_AT_LINE, distance_cm)`; REAR keeps `(ACT_BLIND_REVERSE, v, distance_cm, blind_mode)`. **Contract change, not additive:** the FRONT blind branch (~compiler.py:231-241) currently ignores `blind_mode`; this makes FRONT blind mode-sensitive for the first time. The byte-identical compiler↔sim parity invariant (`test_property_random_walks`) must be re-established jointly with T9 (TC12). Satisfies AC4/AC7 (TC1/TC2). |
| T8 | validator override | T5 | low | `tools/path_planner/validator.py` | Accept `"STOP_AT_LINE"` as a valid `blind_mode` per-edge override; keep rejecting unknowns. Satisfies AC4 (TC4). |
| T9 | sim parity (FRONT now mode-sensitive) | T5 | med | `tools/path_planner/sim/runtime.py` | The FRONT BLIND branch (~runtime.py:766-769) currently emits `ACT_FORWARD` and advances via `on_dead_reckoned_advance`; add the STOP_AT_LINE case emitting `ACT_STOP_AT_LINE` and advancing via `on_intersection` (re-anchor, chassis ends at node), staying byte-identical to T7's compiler output. REAR via `resolve_blind_mode` + the existing blind-reverse path. Satisfies AC9 (TC5/TC6/TC12). |
| T10 | STOP_AT_LINE test fixture (no shipped-field change) | T5 | low | `tools/path_planner/tests/fixtures/` (new fixture edge marked `blind_mode: STOP_AT_LINE`) | Add a **fixture** field with a BLIND edge marked STOP_AT_LINE so the Python tests exercise compiler/validator/sim without touching the shipped `field_data.json` or churning the committed `routes_toy.*` goldens. Per ADR-022 the shipped field source is `field_data.json` (`field_data.py` is a loader); authoring STOP_AT_LINE on a *shipped* edge is supported but deliberately deferred (no shipped edge uses it in this PR). Satisfies AC4. |
| T11 | regenerate routes (enum only) | T6, T7, T8 | low | `WRO2026/routes.h`, `WRO2026/routes.cpp` | Run `uv run python tools/path_planner/build_routes.py`; the only diff is the new `STOP_AT_LINE = 4` in the generated `routes.h` BlindMode enum — no shipped edge uses it, so `routes.cpp` is behavior-identical and shipped goldens do not churn. Generated — do not hand-edit. Satisfies AC4. |
| T12 | PathTracker.h surface | T2 | med | `WRO2026/PathTracker.h` | Add `State::STOP_AT_LINE`; `StopAtLine` field to `NavEntry` + `_activeStopAtLine`; `PathTrackerGoToBuilder::setStopAtLine(StopAtLine)`; extend `_applyGoToBuilder(...)` signature. Satisfies AC5/AC6. |
| T13 | PathTracker.cpp dispatch + re-anchor | T3, T11, T12 | high | `WRO2026/PathTracker.cpp` | In BLIND dispatch: resolve effective mode = call-flag ON/OFF wins over `_resolveBlindMode(edge)` (ON→stop-at-line even on DEFAULT edge; OFF→pipeline blind default; AUTO→edge); when stop-at-line, dispatch `Drive.stopAtLine(edge.distance_cm).setReverse(reverse).async()` and set `State::STOP_AT_LINE`. Add the `STOP_AT_LINE` state: on `Drive.hasFault()`→`_enterFault(DRIVE_FAULT,...)`; on `Drive.isIdle()` success→`Localization.onIntersection(_nextHop)`→`PLAN_NEXT_HOP`. Cache `_activeStopAtLine` at DEQUEUE. Satisfies AC5/AC6/AC7. |
| T14 | Mission flag passthrough | T2, T12 | low | `WRO2026/Mission.h`, `WRO2026/Mission.cpp` | Add `StopAtLine stop_at_line` to `Step`; `MissionClass& setStopAtLine(StopAtLine)` (same `_lastPushOk`/most-recent-GOTO guard as `setRouteDirection`); forward to `PathTracker.goTo(...).setStopAtLine(...)` at GOTO execute. Satisfies AC6. |
| T15 | Python tests | T6, T7, T8, T9, T10, T11 | low | `tools/path_planner/tests/test_compiler.py`, `tools/path_planner/tests/test_header_writer.py`, `tools/path_planner/tests/test_runtime_sim.py`, `tools/path_planner/tests/test_field_data.py`, `tools/path_planner/tests/golden/*` | TC1–TC6 (compiler actions, emitted enum, sim re-anchor + reverse, validator accept) **plus TC12 — re-establish the compiler↔sim byte-parity invariant (`test_property_random_walks`) for STOP_AT_LINE edges** using the T10 fixture. Update goldens deliberately. Satisfies AC4/AC7/AC9. |
| T16 | C++ mirror-slice + arm-math tests | T4, T13 | med | `WRO2026/test/test_path_tracker_stop_at_line.cpp`, `WRO2026/test/test_stop_at_line_armmath.cpp`, `WRO2026/test/CMakeLists.txt` | Following the `test_path_tracker_passthrough.cpp` precedent, add a host target that **inline-mirrors** the T13 STOP_AT_LINE slice (re-anchor-on-success, DRIVE_FAULT-on-no-line, OFF/ON precedence), with every mirrored decision annotated `// MIRROR: PathTracker.cpp:<line>`. **Does NOT link production `PathTracker.cpp` and does NOT mock `Drive.stopAtLine`** (mock `Drive.h` is not extended with a fault surface). Add a second host target (TC11) testing the pure `WROMath.h` arm/debounce free functions directly. Register both in CMake. Satisfies AC5/AC6 (mirrored); AC2 (pure arm-math). |
| T17 | ADR + docs | — | low | `docs/decisions/027-drive-stop-at-line.md`, `DECISIONS.md`, `ARCHITECTURE.md`, `Roadmap.md` | ADR-027: Drive may read the line-sensor leaf for stop-on-line; stop-at-line re-anchors localization; arm-window rule. Add the `Drive → WROLineArray` edge to the ARCHITECTURE.md dependency graph + the `Drive`/`PathTracker` surface rows; index the ADR in DECISIONS.md (+ status table bump); add a Phase-3 Roadmap note + a B3 bench benchmark. Satisfies AC10. |

Parallelizable: T1, T2, T5, T17 have no blockers. T3→T4 (Drive). T5 fans out to T6/T7/T8/T9/T10; T11 converges T6/T7/T8. T12→T13 (PathTracker, also needs T3+T11). T14 after T2+T12. T15 after the Python chain (T6–T11). T16 after T4+T13. **T7 and T9 must land together** (they jointly change the FRONT-blind contract and must keep compiler↔sim parity — TC12).

## Acceptance Criteria

- [ ] **AC1:** `Drive.stopAtLine(expectedCm)` compiles as a `[[nodiscard]]` RAII builder with all knobs in the API Contract; bare `Drive.stopAtLine(40);` drives, detects, overshoots, and brakes (sync-blocking).
- [ ] **AC2:** Detection only arms once `traveled ≥ expectedCm − armWithin`; a line crossed before the window does **not** stop the chassis.
- [ ] **AC3:** On a `T_OR_CROSS` detection the chassis rolls forward `overshoot` cm then active-brakes; `isLineFound()` returns `true`. *(Physical stop accuracy verified by bench benchmark B3.7; the detection-arming precondition is verified host-side by TC11.)*
- [ ] **AC4:** `BlindMode::STOP_AT_LINE` exists in `pipeline_config.py`, the generated `routes.h`, the compiler, and the validator; a `field_data` edge can be authored with it and `build_routes.py` regenerates cleanly.
- [ ] **AC5:** A PathTracker blind hop resolving to STOP_AT_LINE dispatches `Drive.stopAtLine(edge.distance_cm).async()` and, on success, advances `Localization` via `onIntersection()`. *(Re-anchor here means updating the discrete `_currentNode` id only — heading remains IMU-live by design, per ADR-005. It does not correct yaw/xy.)*
- [ ] **AC6:** `goTo().setStopAtLine(ON)` forces stop-at-line on a blind hop, `OFF` force-disables it even when the edge says STOP_AT_LINE, and `AUTO` defers to the edge; `Mission.setStopAtLine(...)` forwards the same.
- [ ] **AC7:** If no line is detected by `expectedCm + STOP_AT_LINE_MAX_MARGIN_CM`, Drive faults `LINE_NOT_FOUND` and PathTracker surfaces `DRIVE_FAULT` (no dead-reckon fallback).
- [ ] **AC8:** Reverse stop-at-line drives backward and reads the rear board when `LINE_BOARD_COUNT == 2`; on a 1-board chassis the bare reverse verb is rejected rather than reading the wrong board.
- [ ] **AC9:** The Python sim treats a `STOP_AT_LINE` blind edge as a node re-anchor (`on_intersection`); goal-position cross-checks pass.
- [ ] **AC10:** ADR-027 is written and indexed; ARCHITECTURE.md shows the new `Drive → WROLineArray` dependency; `uv run pytest tools/path_planner/tests/` is green and the CI C++ harness passes.

## Notes for Implementer

- **Tick order matters (no extra I2C):** `cooperativeTick()` runs `LineArray.update()` → `Drive.update()` → `LineFollower.update()`. When `Drive._updateStopAtLine()` reads `LineArray.getJunctionType()`, the sample is current (cache-mode refreshes at the top; legacy mode refreshes inline). During a sync-blocking `stopAtLine`, `_drainBlocking()` re-enters the registered `cooperativeTick`, so the sensor keeps refreshing — do **not** add a second read path.
- **Do not consume `Intersection.isAtIntersection()`** from Drive — it is read-and-clear and owned by PathTracker. Read `LineArray.getJunctionType()` directly. No host test guards this (the mirror harness does not model `Intersection`), so it is a **required code-review check** in T4 — a regression that consumed the edge would silently steal PathTracker's nav arming bit.
- **Overshoot ramp-down:** the overshoot distance is known, so ramp cruise → `PWM_MIN_MAGNITUDE` over it then active-brake (reuse the existing smooth-stop helper / pattern, not a coast). The *seek* phase cannot ramp down (stop point unknown until detection) — cruise flat there.
- **GYRO heading:** default to the current held yaw; PathTracker passes `_resolvedBodyHeading` via `.setHeading(Localization.toImuYaw(...))` exactly as the existing GYRO blind branch does. `SYNC` (the default) needs no heading.
- **Reverse + rear board:** set `LineArray.setReverseActive(true)` for the duration and restore to `false` (front) on DONE/fault. LineFollower is disabled during the move, so Drive briefly owning the active-board selection is safe. In reverse, `expectedCm`, `armWithin`, and the overshoot are all measured along the *backward* travel direction (the overshoot continues backward past the line). Mark the reverse path `@note untested on hardware until the rear board is wired` (ADR-024 wiring still pending).
- **Compiler↔sim FRONT parity is now load-bearing:** before this PR the FRONT blind path ignored `blind_mode`; T7/T9 make it mode-sensitive. They must be implemented and reviewed together, and TC12 (`test_property_random_walks`) must stay green — a divergence between the compiler's and sim's STOP_AT_LINE action streams silently breaks the sim-mirrors-robot guarantee.
- **Stale doc note (non-blocking):** `Config::LINE_ARRAY_TICK_CACHE_ENABLED` is currently `true`, but `WROLineArray.h`'s docstring still calls `false` the "default — current PR #20 behaviour." The tick-order reasoning above relies on cache-mode being on (it is). If you touch the tick path, fix that one-line doc/Config drift in passing.
- **Phase context:** this extends existing modules (no new module), so it respects Phase 3's "no new modules" rule. Add a Phase-3 bench benchmark (e.g. **B3.7**: `Drive.stopAtLine` stops within ±2 cm of a taped perpendicular line over 10 trials, no false-trigger on an intervening line) — the real Drive state machine is verified on hardware there, not in CI.
- **Goldens:** prefer adding a *fixture* edge marked `STOP_AT_LINE` for tests over mutating the shipped toy `field_data` if that would churn the committed `routes_toy.*` goldens unnecessarily; if the shipped field is changed, regenerate and review `map.png` + goldens in the same commit.
- **Rollback:** the change is additive (a new enum value defaulting off, a new verb). Reverting = drop the new BlindMode value (no edge uses it by default), the Drive verb, and the PathTracker state; existing blind/projection/line behavior is untouched.
