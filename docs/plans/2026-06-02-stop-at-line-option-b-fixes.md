# Stop-at-Line — Option B + Review-Findings Fix Plan

**Goal:** Make `Drive.stopAtLine()` a uniform, composable command that behaves identically across direction (forward/reverse), drive mode, and the firmware ↔ Python-sim ↔ compiler boundary — and fix all 13 findings from the `/code-review max` of commit `a2535e7`.

**Approach (Option B + unified resolver):** Introduce one canonical decision — `resolveStopAtLine(...)` — frozen as a reviewed truth-table fixture (T0) and transcribed verbatim into a header-only `WRO2026/StopAtLinePolicy.h` (C++, mirroring `DirectionPolicy.h`) and a `sim/runtime.py` twin (Python). It encodes a single rule — *stop-at-line is effective iff a line board faces the travel direction (front always; rear only when `LINE_BOARD_COUNT >= 2`)* — and returns the effective motion mode. The two transcriptions are gated by a two-sided parity test (TC13), the same way `resolveDirection`/TC18 already work in this repo. Around that, harden the seek state machine (SEEK max-distance + a no-progress stall watchdog, both extracted to pure host-testable predicates), honor the edge's drive mode, and apply the cleanup/altitude fixes.

**Attribution (do not conflate):** The resolver does **not** by itself fix findings #1 and #2 — split them:
- **#2 (1-board turn-then-fault)** is a genuine *firmware* bug the resolver fixes by degrading STOP_AT_LINE before dispatch (T1/T5).
- **#1 (2-board firmware↔offline divergence)** is **not** a firmware bug — the firmware 2-board REAR reverse-seek already exists and works (`PathTracker.cpp:1012` already dispatches `Drive.stopAtLine(distance).setReverse(reverse).async()`). The real defect is offline: `compiler.py:229` and `sim/runtime.py:773` always emit `ACT_BLIND_REVERSE` + dead-reckon for a REAR blind hop, ignoring STOP_AT_LINE mode (unlike their own FRONT branches, which already special-case it). The load-bearing fix for #1 is the new mode-sensitive sim/compiler emission (T6/T7). **AC1 (firmware↔offline agreement) is the primary check for #1.**

## Findings under fix (from `/code-review max` of `a2535e7`)

All 13 are reproduced so coverage is auditable. Severity is the reviewer's.

| # | Finding (verbatim summary) | Sev | Fixed by | Verified by |
|---|----------------------------|-----|----------|-------------|
| 1 | A REAR-resolved BLIND stop-at-line hop dispatches a reverse seek + `onIntersection` on the robot, but the sim/compiler emit `ACT_BLIND_REVERSE` + dead-reckon for the same hop — firmware≠offline. | high | T0,T6,T7 | AC1, AC3 |
| 2 | On a 1-board chassis a forced-REAR stop-at-line hop slips past the BLIND-exempt `REVERSE_UNAVAILABLE` guard, turns 180°, then Drive rejects reverse stop-at-line → `LINE_NOT_FOUND`/`DRIVE_FAULT` (robot ends backward, stuck). | high | T0,T1,T5 | AC2 |
| 3 | The SEEK state has no max-distance fault; only ARMED checks `traveled >= maxCm`, so a seek that arms late (tight `setMaxDistance`) overshoots its cap, and one that never advances (zero cruise / stalled wheel) hangs the async FSM forever. | med | T2,T4 | AC6 |
| 4 | The stop-at-line dispatch always issues a default `SYNC` seek and discards the edge's resolved BlindMode/heading, so forcing stop-at-line onto a `GYRO_TSHAPE` edge silently drops yaw-lock. | med | T1,T5 | AC5 |
| 5 | Every forward stop-at-line hop drives over its target `T_OR_CROSS` while `Intersection.update()` independently counts it, inflating `getEdgeCount()` and risking a stale `_edgeFired` leak into the next LINE hop. | med | T5 | AC7 |
| 6 | The StopAtLine ON/OFF/AUTO precedence is hand-inlined in `PathTracker.cpp:986` and byte-copied into the test's `replica_resolveStopAtLine`, instead of a shared header like `DirectionPolicy.h` — the mirror can drift and is the only coverage. | med | T0,T1,T8 | AC4 |
| 7 | The `driveContinuous` lambda re-implements the RAW/SYNC/GYRO continuous-emit switch already in `_updateProfile`; they should share one `_emitContinuous(mode,mag,heading)`. | med | T4 | AC8 |
| 8 | `STOP_AT_LINE_T_OR_CROSS_RAW = 3` hard-duplicates `JunctionType::T_OR_CROSS`'s value with a manual two-site-drift comment and no compile-time link. | med | T3,T4 | AC9 |
| 9 | `_armWithinSet`/`_overshootSet` shadow flags are dead — the member default and the ternary fallback resolve to the same Config constant. | med | T4 | AC10 |
| 10 | The reverse seek is plumbed through builder+DriveCommand+`_dispatchNext`+`_updateStopAtLine`+PathTracker yet rejected at runtime on the only built (1-board) chassis and untested on 2-board; its board-restore code is interleaved into every live exit as if reachable. | low | T5,T6,T7,T12 | AC1, AC2 |
| 11 | DONE and the no-line fault path repeat the same teardown trio (`setReverseActive(false)`+IDLE+`_moveDone`+`_turnDone`), with two more copies in `stop()`/`_drainBlocking` — extract one helper. | low | T4 | AC10 |
| 12 | `_updateStopAtLine` reads `MiniR4.M2.getDegrees()` (STM32 round-trip) + `encDegToCm` every tick including DONE, which uses neither. | low | T4 | AC10 |
| 13 | The STOP_AT_LINE dispatch sets `rc = 0x00` then falls into the generic `if (rc != 0x00)` tail that can never fire (its only failure returns early). | low | T4 | AC10 |

## Scope

- **In scope:**
  - `T0` canonical `resolveStopAtLine` truth-table fixture (single normative source) + new `WRO2026/StopAtLinePolicy.h` (C++ transcription) + `resolve_stop_at_line` (Python transcription). Replaces the inlined switch in `PathTracker.cpp` and the `replica_resolveStopAtLine` in the host test. [#1, #2, #6; subsumes the `edgeBlindMode`/`resolvedBlindMode` dance.]
  - Option B: REAR stop-at-line via the rear board on a 2-board chassis, in firmware **and** sim **and** compiler, byte-parity preserved. New `ACT_STOP_AT_LINE_REVERSE` primitive. [#1, #2, #10.]
  - 1-board forced-REAR stop-at-line degrades to a **reverse** dead-reckoned blind move (keeps the resolved REAR direction; drops the seek/re-anchor) — mirrors the sim's existing `ACT_BLIND_REVERSE`. [#1, #2.]
  - Seek drive mode: default `SYNC`, but a call-site `ON` forced onto a `GYRO_TSHAPE`/`RAW_FLAT` edge drives in that edge's mode (+ resolved body heading for `GYRO`). [#4.]
  - SEEK-state safety as **pure host-testable predicates** (`stopAtLineSeekMaxedOut`, `stopAtLineProgressed`, `stopAtLineStalled` in `WROMath.h`): SEEK honors `maxCm`, and a `millis()` no-progress watchdog (`Config::STOP_AT_LINE_STALL_TIMEOUT_MS`) faults a never-advancing seek. [#3.]
  - Intersection drain on stop-at-line dispatch so the seek's physical crossing cannot leak a stale nav edge into the next LINE hop. [#5.]
  - Drive cleanups: `_emitContinuous(mode,mag,heading)` shared with `_updateProfile` [#7]; typed `T_OR_CROSS` constant on the `WROLineArray` leaf [#8]; remove dead `_armWithinSet`/`_overshootSet` [#9]; `_endStopAtLine(...)` teardown helper [#11]; skip the encoder read in DONE [#12]; remove dead `rc = 0x00` [#13]; drop the redundant `_stopDetectStreak = 0` in SEEK→ARMED.
  - Host tests (C++ + Python) for the resolver truth table, REAR/board-count degrade, mode honoring, SEEK max + watchdog predicates; sim/compiler parity tests.
  - Docs: amend ADR-027 (Option B, resolver, watchdog, seek mode, **and a "Seek state machine" subsection answering "what is SEEK"**) + ADR-024 cross-ref; update DECISIONS.md, ARCHITECTURE.md, Roadmap.md (B3.7).
- **Out of scope (explicitly):**
  - **`STOP_AT_LINE` on `LINE`/`PROJECTION` edges.** The user's directive is "apply to EVERY thing *that should work*." A `LINE`/`PROJECTION` edge already follows a line and ends on a physical crossing (the line follower stops there), so stop-at-line is redundant there — ADR-027 scopes it to BLIND edges. This is the one place the "everything" directive is intentionally bounded; flagged here for the user to object at approval if they meant otherwise.
  - Physical rear-board hardware bench test (B3.7-reverse) — deferred until the rear board is wired; logic is CI-tested only (no host C++ compiler locally — every C++ AC is CI-gated, not locally runnable).
  - An async stall watchdog for `_updateTurnProfile` — same class of gap, pre-existing, **follow-up** (a jammed `.async()` turn can still hang; tracked, not fixed here).
  - Migrating `Intersection.cpp`'s hand-rolled streak to `debounceStep` (not one of the 13; its dual-streak is richer — leave).
  - Opportunistic mid-blind re-anchoring — rejected by ADR-027.
  - Regenerating `WRO2026/routes.h` / changing production `field_data.py` — fixtures cover the tests.

## Alternatives considered

- **Minimal fix (rejected):** just mirror the firmware in the sim/compiler REAR-blind branch (~10 lines) + the cleanups, *without* a new `StopAtLinePolicy.h`. Cheaper, but leaves the precedence logic inlined in `PathTracker.cpp` and the `replica_resolveStopAtLine` test drift (#6) unaddressed, and does not give the user the "works the same way everywhere" uniform command. **The user explicitly chose the unified-resolver approach (new `StopAtLinePolicy.h`).** The new header is justified by killing the test replica and matching the established `DirectionPolicy.h` altitude — not speculative abstraction.

## Decisions

- **One rule, one place:** stop-at-line effectiveness = `seekRequested AND boardFacingTravelExists`. Lives in the T0 fixture and is transcribed to `resolveStopAtLine` (C++) and `resolve_stop_at_line` (Python); parity enforced like `resolveDirection`/TC18.
- **Findings embedded verbatim, not by number.** The 13 findings are carried inline with a finding→task→AC table; bare "#N" references aren't enough to confirm "fix ALL."
- **Stop-at-line stays BLIND-only.** LINE/PROJECTION already stop on crossings via line-following; extending the verb there is redundant (ADR-027). Surfaced above for user objection.
- **1-board REAR → reverse dead-reckon, not fault, not forward-flip.** Reverse dead-reckon needs no sensor and is already permitted on 1-board; matches the sim's `ACT_BLIND_REVERSE` and avoids the turn-then-hard-fault. Confirmed with user.
- **Seek mode = SYNC default, honor edge mode on `ON`-override.** Confirmed with user.
- **SEEK gets distance check + stall watchdog**, both as pure predicates so the safety logic is CI-covered (the seek is the only unbounded async motion, and PathTracker dispatches it `.async()` with no drain timeout). Confirmed with user.
- **Drive's 1-board reverse-reject stays as defense-in-depth** (now unreachable from PathTracker, which degrades first, but guards the bare verb).
- **ADR-027 amended in place** (not superseded); reverse moves from "untested/deferred" to "CI-tested logic; physical bench deferred."

## Acceptance Criteria

- [ ] **AC1 (PRIMARY for #1):** On a 2-board chassis, a REAR-resolved `STOP_AT_LINE` blind hop produces *byte-identical agreement* between firmware intent and the offline streams: firmware runs a reverse seek + `Localization.onIntersection`; compiler and sim emit `ACT_STOP_AT_LINE_REVERSE` and re-anchor via `on_intersection`.
- [ ] **AC2 (#2):** On a 1-board chassis, a forced-REAR `STOP_AT_LINE` blind hop degrades to a reverse dead-reckon (`ACT_BLIND_REVERSE` + `on_dead_reckoned_advance`) with **no** fault and **no** 180°-then-fault; firmware mirrors this (resolver returns `seek=false`).
- [ ] **AC3 (#1):** A FORWARD `STOP_AT_LINE` blind hop is unchanged — `ACT_STOP_AT_LINE` + `onIntersection`/`on_intersection`.
- [ ] **AC4 (#6):** `PathTracker.cpp` and the host test both `#include "StopAtLinePolicy.h"` and call `resolveStopAtLine`; `replica_resolveStopAtLine` is deleted. TC13 asserts the C++ and Python resolvers produce identical output for every T0 row.
- [ ] **AC5 (#4):** An `ON` override on a `GYRO_TSHAPE` edge dispatches `.setMode(GYRO).setHeading(...)`; on `RAW_FLAT`, `RAW`; on `STOP_AT_LINE`/`SYNC_TSHAPE`/`DEFAULT`, `SYNC`.
- [ ] **AC6 (#3):** Pure predicates make the safety logic CI-verifiable: `stopAtLineSeekMaxedOut(traveled,maxCm)` faults at/above `maxCm` (TC16); the watchdog pair `stopAtLineProgressed`+`stopAtLineStalled` trips `LINE_NOT_FOUND` on a never-advancing seek and does **not** trip on a slow-but-moving 100 cm seek (TC17).
- [ ] **AC7 (#5):** After a forward stop-at-line hop, no stale `Intersection` edge causes a false intersection on the following LINE hop (the dispatch drains `Intersection.isAtIntersection()`).
- [ ] **AC8 (#7):** `_updateStopAtLine` and `_updateProfile` both emit continuous PWM through one `_emitContinuous(...)`; no duplicated RAW/SYNC/GYRO switch.
- [ ] **AC9 (#8):** `Drive.cpp` references a named `T_OR_CROSS` constant owned by `WROLineArray`, not a bare `3`.
- [ ] **AC10 (#9/#11/#12/#13):** `_armWithinSet`/`_overshootSet` removed; one `_endStopAtLine` used by DONE/fault/`stop()`/`_drainBlocking`; no encoder read in DONE; no dead `rc = 0x00`.
- [ ] **AC11:** `uv run pytest tools/path_planner/tests/` passes; the C++ host test suite builds and passes in CI (not locally runnable — no host C++ compiler).
- [ ] **AC12:** Every one of the 13 findings has a verbatim entry in the "Findings under fix" table with non-empty Fixed-by/Verified-by; no finding (esp. #10) is referenced without a description.
- [ ] **AC13 (user ask "Explain what is SEEK state"):** ADR-027 contains a "Seek state machine" subsection defining SEEK, ARMED, OVERSHOOT, DONE, each transition guard (arm window, `T_OR_CROSS` debounce, overshoot, brake), and the max-distance/stall-watchdog fault exits.

## Data Model

### `resolveStopAtLine` contract — the T0 canonical truth table (normative source)

`edgeMode` is the per-edge BlindMode **after** `_resolveBlindMode` (DEFAULT already → pipeline blind). `direction` is the **post-`resolveDirection`** value, so on a 1-board chassis REAR only occurs via an operator force (auto already short-circuits to FRONT — `DirectionPolicy.h:65`).

```
seekRequested = (callFlag == ON) || (callFlag == AUTO && edgeMode == STOP_AT_LINE)   // OFF → false
boardOK       = (direction == FRONT) || (lineBoardCount >= 2)
seek          = seekRequested && boardOK

if seek:
    motionMode = (edgeMode == STOP_AT_LINE) ? SYNC_TSHAPE : edgeMode      // ON onto a typed edge keeps its mode
else:
    base       = (edgeMode == STOP_AT_LINE) ? pipelineBlind : edgeMode    // OFF or board-degrade → dead-reckon
    motionMode = (base == STOP_AT_LINE) ? SYNC_TSHAPE : base              // defensive: pipeline==STOP_AT_LINE
```

T0 freezes this as `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv` — columns `callFlag,edgeMode,pipelineBlind,direction,lineBoardCount,expect_seek,expect_motionMode` — enumerating every input combination. T1 (C++) and T6 (Python) each transcribe this fixture; they do **not** read from each other. TC13 drives both against the fixture and fails CI on any divergence.

```cpp
// WRO2026/StopAtLinePolicy.h  (header-only inline; includes Config.h, NavTypes.h, routes.h — NOT Drive.h)
struct StopAtLineResolution {
    bool              seek;        // true → dispatch the Drive.stopAtLine seek
    Routes::BlindMode motionMode;  // motion bundle; NEVER STOP_AT_LINE or DEFAULT
};
inline StopAtLineResolution resolveStopAtLine(
    StopAtLine callFlag, Routes::BlindMode edgeMode, Routes::BlindMode pipelineBlind,
    Direction direction, uint8_t lineBoardCount);
```

### PathTracker dispatch mapping

`blindModeToDriveMode(BlindMode) -> DriveMode` is a **new** small helper (`SYNC_TSHAPE→SYNC`, `GYRO_TSHAPE→GYRO`, `RAW_FLAT→RAW`). The existing dead-reckon `switch(resolvedBlindMode)` at `PathTracker.cpp:1085-1133` encodes the same mapping inline *but bundled with full builder chains* (GYRO sets heading, RAW pins start/end speeds) — so this is genuinely new extraction, not a drop-in reuse.

```cpp
const StopAtLineResolution r = resolveStopAtLine(
    _activeStopAtLine, _resolveBlindMode(edge), Routes::PIPELINE_CONFIG.blind,
    _resolvedDirection, Config::LINE_BOARD_COUNT);
if (r.seek) {
    if (_passThrough) LineFollower.setEnabled(false).setBrake(false);
    (void) Intersection.isAtIntersection();                 // drain stale edge (AC7)
    const DriveMode m = blindModeToDriveMode(r.motionMode);
    auto b = Drive.stopAtLine(edge.distance_cm).setReverse(reverse).setMode(m);
    if (m == DriveMode::GYRO) b.setHeading(Localization.toImuYaw(_resolvedBodyHeading));
    b.async();
    _state = State::STOP_AT_LINE; _passThrough = false; break;
}
// else: dead-reckon using r.motionMode through the existing SYNC_TSHAPE/GYRO_TSHAPE/RAW_FLAT switch.
```

### Pure seek predicates — `WROMath.h` (host-testable; mirror the `stopAtLineArmed`/`debounceStep` docblock convention)

```cpp
inline bool stopAtLineSeekMaxedOut(float traveledCm, float maxCm);                       // traveledCm >= maxCm
inline bool stopAtLineProgressed(float traveledCm, float lastProgressCm, float epsCm);   // (traveled-last) >= eps
inline bool stopAtLineStalled(uint32_t nowMs, uint32_t lastProgressMs, uint32_t toMs);   // (now-last) >= toMs
```

`_updateStopAtLine` each tick: if `stopAtLineProgressed(...)` → update `_stopLastProgressCm = traveled; _stopLastProgressMs = millis()`. Fault `LINE_NOT_FOUND` if `stopAtLineSeekMaxedOut(...)` (in SEEK and ARMED) **or** `stopAtLineStalled(...)`. The progress decision lives in the predicate so the "slow-but-moving never trips" case (AC6) is CI-verifiable.

### New Drive state — `Drive.h`

```cpp
uint32_t _stopLastProgressMs = 0;   // millis() at last progress (watchdog reset point)
float    _stopLastProgressCm = 0.0f;// traveled at last progress (progress detector baseline)
```

### New primitive — `tools/path_planner/sim/runtime.py`

```python
ACT_STOP_AT_LINE_REVERSE = "STOP_AT_LINE_REVERSE"   # (action, target_node, distance_cm, blind_mode)
# Mirrors ACT_BLIND_REVERSE's 4-tuple; emitted for a 2-board REAR stop-at-line hop; re-anchors via on_intersection.
```

### New constants — `Config.h`

```cpp
// No-progress watchdog for the unbounded stopAtLine seek. The cooperative tick runs every ~10-30 ms
// (LineArray I2C refresh + the 2 ms drain delay), so a moving seek advances every tick; 1500 ms (~50-150
// idle ticks) trips only a truly stalled/zero-power seek, never a slow-but-moving one. Bench-tunable.
constexpr uint32_t STOP_AT_LINE_STALL_TIMEOUT_MS  = 1500;
// Minimum traveled advance (cm) per check that counts as progress (guards float noise / encoder jitter).
constexpr float    STOP_AT_LINE_PROGRESS_EPS_CM   = 0.2f;
```

## Error Handling

- **No line by `maxCm` (SEEK or ARMED):** `Fault::LINE_NOT_FOUND` → `_endStopAtLine` (stop + active brake + clear queue + restore front board). PathTracker surfaces `DRIVE_FAULT` via the top-of-update guard.
- **No progress for `STOP_AT_LINE_STALL_TIMEOUT_MS`:** same `LINE_NOT_FOUND` path (closes the async no-timeout hole).
- **REAR stop-at-line with no rear board:** never dispatched as a seek (resolver `seek=false` → reverse dead-reckon). Drive's bare-verb reverse reject remains as defense-in-depth.
- **Pass-through LINE→BLIND into a stop-at-line hop:** release LineFollower (no brake) before handing wheels to the seek; `_passThrough` consumed.

## Testing Strategy

**Levels:** Unit (C++ host + Python), Integration (Python sim end-to-end). PWM/encoder/brake actuation stays bench-only (B3.7); the *decisions* (resolver, arm window, debounce, max-distance, stall watchdog) are all pure predicates and CI-covered.

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | resolver: AUTO + STOP_AT_LINE edge, FRONT, 1-board | Unit (C++ & Py) | `seek=true`, `SYNC_TSHAPE` |
| TC2 | resolver: AUTO + STOP_AT_LINE edge, REAR, 2-board | Unit (C++ & Py) | `seek=true`, `SYNC_TSHAPE` |
| TC3 | resolver: **forced-REAR** (edge override) + STOP_AT_LINE, 1-board (the only way REAR+1-board occurs — auto short-circuits to FRONT upstream) | Unit (C++ & Py) | `seek=false`, `motionMode=pipelineBlind` (degrade) |
| TC4 | resolver: ON + GYRO_TSHAPE edge, FRONT, 1-board | Unit (C++ & Py) | `seek=true`, `GYRO_TSHAPE` |
| TC5 | resolver: ON + RAW_FLAT edge, REAR, 2-board | Unit (C++ & Py) | `seek=true`, `RAW_FLAT` |
| TC6 | resolver: OFF + STOP_AT_LINE edge, FRONT | Unit (C++ & Py) | `seek=false`, `pipelineBlind` |
| TC7 | resolver: AUTO + SYNC_TSHAPE edge, FRONT | Unit (C++ & Py) | `seek=false`, `SYNC_TSHAPE` |
| TC8 | compiler: 2-board REAR STOP_AT_LINE edge | Unit (Py) | `(ACT_STOP_AT_LINE_REVERSE, v, dist, STOP_AT_LINE)` |
| TC9 | compiler: **same fixture** at 2-board vs 1-board forced-REAR | Unit (Py) | 2-board → `ACT_STOP_AT_LINE_REVERSE`; 1-board → `ACT_BLIND_REVERSE` (asserts the branch divergence, not a tautology) |
| TC10 | sim go_to: 2-board REAR STOP_AT_LINE hop | Integration (Py) | `on_intersection` re-anchor; ends on node; compiler↔sim byte-identical |
| TC11 | sim go_to: **same fixture** 2-board vs 1-board forced-REAR | Integration (Py) | 2-board → `on_intersection`; 1-board → `on_dead_reckoned_advance`; each byte-identical to compiler |
| TC12 | compiler↔sim parity over a FORWARD STOP_AT_LINE hop (regression) | Integration (Py) | unchanged `ACT_STOP_AT_LINE` in both |
| TC13 | **C++ ↔ Py resolver parity over the T0 fixture (every row)** — required merge gate for T1/T6/T7 | Unit | identical `{seek, motionMode}` both languages |
| TC14 | PathTracker STOP_AT_LINE success/fault slice (existing) | Unit (C++) | re-anchor on idle; DRIVE_FAULT on Drive fault |
| TC15 | `stopAtLineArmed` / `debounceStep` pure math (existing) | Unit (C++) | unchanged |
| TC16 | `stopAtLineSeekMaxedOut`: true at/above `maxCm`, false below | Unit (C++) | correct boundary |
| TC17 | watchdog pair: (a) 0.5 cm/tick advance over 100 cm with sub-step eps → `stopAtLineProgressed` true each tick, `lastProgressMs` keeps resetting, `stopAtLineStalled` never trips; (b) flat `traveled` → `progressed` false, `stalled` trips once `now-last >= timeout` | Unit (C++) | no-trip while moving; trip when stalled |
| TC18 | PathTracker slice: stop-at-line dispatch calls `Intersection.isAtIntersection()` (drain) before entering STOP_AT_LINE (mirror-slice assertion; full drain behavior is bench-verified) | Unit (C++) | drain ordering present |

**Test data:** Python — `nx.DiGraph` fixtures with `primitive_mode=BlindMode.STOP_AT_LINE` + `direction_override`, `line_board_count` param (existing pattern); the T0 CSV for TC13. C++ — table-driven over `resolveStopAtLine` and the `WROMath.h` predicates (header-only, no linkage).
**Run command:** `uv run pytest tools/path_planner/tests/` (Python); CI ctest harness (C++).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T0 | Freeze resolver truth-table fixture | — | low | `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv` | Author the canonical truth table (every `callFlag×edgeMode×pipelineBlind×direction×lineBoardCount` → `{seek, motionMode}`) per the Data-Model rule. This is the one normative source T1 and T6 transcribe (not from each other). Exit: reviewed + merged before T1/T6 start. Satisfies AC4 basis. |
| T1 | Create `StopAtLinePolicy.h` | T0 | high | `WRO2026/StopAtLinePolicy.h` | Header-only `inline resolveStopAtLine(...)` + `StopAtLineResolution`, transcribing the T0 fixture verbatim. Doxygen mirroring `DirectionPolicy.h` incl. the "keep step-for-step identical with `sim/runtime.py::resolve_stop_at_line`; T0 is the source; TC13 is the gate" note. Include only `Config.h`, `NavTypes.h`, `routes.h`. Satisfies AC4. |
| T2 | Config constants | — | low | `WRO2026/Config.h` | Add `STOP_AT_LINE_STALL_TIMEOUT_MS = 1500` and `STOP_AT_LINE_PROGRESS_EPS_CM = 0.2f` in the stop-at-line block with the Data-Model docstrings. Satisfies AC6. |
| T3 | Typed `T_OR_CROSS` on the leaf | — | low | `WRO2026/WROLineArray.h` | Add `static constexpr uint8_t JUNCTION_T_OR_CROSS = 3;` (named) to `WROLineArrayClass` documenting the `getJunctionType()` 0..3 contract, so consumers stop hardcoding `3`. Satisfies AC9. |
| T4 | Seek hardening + cleanups | T2, T3 | high | `WRO2026/WROMath.h`, `WRO2026/Drive.h`, `WRO2026/Drive.cpp` | Add the three pure predicates to `WROMath.h`. In `_updateStopAtLine`: add `_stopLastProgressCm`/`_stopLastProgressMs` members, per-tick progress update via `stopAtLineProgressed`, SEEK+ARMED `stopAtLineSeekMaxedOut` fault, and the `stopAtLineStalled` watchdog → `LINE_NOT_FOUND`; skip the encoder read in DONE; drop the redundant `_stopDetectStreak=0` in SEEK→ARMED (note ARMED is the only writer). Extract `_emitContinuous(DriveMode,int16_t,int16_t)` (reads **no** instance state — params only) shared with `_updateProfile`. Extract `_endStopAtLine(bool brake)` used by DONE/fault/`stop()`/`_drainBlocking`. Use `WROLineArrayClass::JUNCTION_T_OR_CROSS`. Remove `rc=0x00` in the STOP_AT_LINE dispatch case. In `StopAtLineBuilder` remove `_armWithinSet`/`_overshootSet` only (their member defaults already equal the Config fallback); **keep** `_maxDistanceSet`/`_headingSet`/`_powerExplicit`/`_presetSet` (their defaults aren't the resolved value). Satisfies AC6, AC8, AC9, AC10. |
| T5 | PathTracker resolver dispatch | T1 | high | `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h` | Replace the inlined precedence switch + `edgeBlindMode`/`resolvedBlindMode` dance with `resolveStopAtLine(...)`; on `r.seek` dispatch `Drive.stopAtLine(edge.distance_cm).setReverse(reverse).setMode(blindModeToDriveMode(r.motionMode))` (+ `.setHeading(toImuYaw(_resolvedBodyHeading))` when GYRO), draining `Intersection.isAtIntersection()` first; else dead-reckon via the existing switch on `r.motionMode`. Add the new `blindModeToDriveMode` helper. Satisfies AC1–AC5, AC7. |
| T6 | Python sim resolver + reverse stop-at-line | T0 | med | `tools/path_planner/sim/runtime.py` | Add `resolve_stop_at_line(...)` transcribing the T0 fixture (NOT reading T1) + `ACT_STOP_AT_LINE_REVERSE` (+ `__all__`). In `go_to` and `step_through` EDGE_BLIND: compute `(seek, motion_mode)`; FORWARD+seek → `ACT_STOP_AT_LINE`+`on_intersection`; REAR+seek → `ACT_STOP_AT_LINE_REVERSE`+`on_intersection`; else direction-appropriate dead-reckon + `on_dead_reckoned_advance`. Use `LINE_BOARD_COUNT`. Satisfies AC1–AC3. |
| T7 | Compiler resolver + reverse stop-at-line | T6 | med | `tools/path_planner/compiler.py` | Import `resolve_stop_at_line`, `ACT_STOP_AT_LINE_REVERSE` from sim runtime; in `compile_path` EDGE_BLIND (FRONT and REAR) emit per the resolver, byte-parity with T6. Satisfies AC1–AC3. |
| T8 | C++ host tests: resolver + slice + drain | T1, T5 | low | `WRO2026/test/test_path_tracker_stop_at_line.cpp` | Delete `replica_resolveStopAtLine`; `#include "StopAtLinePolicy.h"`; assert TC1–TC7 + the TC13 C++ side against the real `resolveStopAtLine`; keep TC14; add TC18 drain-ordering mirror-slice. (Resolver assertions depend only on T1; slice/drain on T5 — same file, so combined.) Satisfies AC4, AC5, AC7, AC12-basis. |
| T9 | C++ predicate tests | T4 | low | `WRO2026/test/test_stop_at_line_armmath.cpp` | Add TC16 (`stopAtLineSeekMaxedOut`) and TC17 (`stopAtLineProgressed`+`stopAtLineStalled` watchdog pair — slow-moving no-trip; never-advancing trip). Satisfies AC6. |
| T10 | Python compiler tests | T7 | low | `tools/path_planner/tests/test_compiler.py` | Add TC8, TC9 (2-board vs 1-board same-fixture divergence), TC12; assert `ACT_STOP_AT_LINE_REVERSE` shape. Satisfies AC1, AC2, AC3. |
| T11 | Python sim + parity tests | T6 | low | `tools/path_planner/tests/test_runtime_sim.py` | Add TC10, TC11 (2-board re-anchor vs 1-board dead-reckon, same fixture), and the TC13 Python side over the T0 CSV. Satisfies AC1, AC2, AC4. |
| T12 | Docs | T8, T9, T10, T11 | low | `docs/decisions/027-drive-stop-at-line.md`, `docs/decisions/024-bidirectional-line-following.md`, `DECISIONS.md`, `ARCHITECTURE.md`, `Roadmap.md` | Amend ADR-027 (Option B, unified resolver, watchdog, seek-mode rule; reverse → "CI-tested logic, bench deferred") **and add a "Seek state machine" subsection** defining SEEK/ARMED/OVERSHOOT/DONE + the new fault exits (answers "Explain what is SEEK state", AC13); ADR-024 cross-ref to the resolver; refresh DECISIONS.md/ARCHITECTURE.md stop-at-line surface; update Roadmap B3.7 (reverse logic CI-covered) and add the `_updateTurnProfile` watchdog follow-up. Written after the impl+tests land so it documents the verified state. Satisfies AC13. |

## Notes for Implementer

- **Parity is load-bearing:** T0 is the single source; T1 (C++) and T6 (Python) transcribe it independently and in parallel; T7 inherits via T6. TC13 (every T0 row, both languages) is a required merge gate. Existing guardrails: TC14c (`LINE_BOARD_COUNT` grep) and TC18 (`resolveDirection` parity) — keep the new resolver's cross-language note pointing both ways.
- **`_resolveBlindMode` already maps `DEFAULT`→pipeline blind** (`PathTracker.cpp:43-48`) — so the resolver's `edgeMode` input is never `DEFAULT` when T5 passes `_resolveBlindMode(edge)`.
- **`direction` is the post-`resolveDirection` value.** On 1-board, REAR reaches the resolver only via an operator force (auto returns FRONT — `DirectionPolicy.h:65`); TC3 is authored as a forced-REAR case for that reason.
- **Watchdog correctness hinges on progress detection**, not the timeout subtraction: update `_stopLastProgressCm`/`_stopLastProgressMs` whenever `stopAtLineProgressed(...)` is true, initialized at dispatch. This is why the progress comparison is its own predicate (TC17 covers the slow-moving-100 cm no-trip case).
- **`_emitContinuous` extraction (T4):** the helper takes mode+mag+heading as params and reads NO instance state, so `_updateProfile` (`_profileMode`/`_profileHeading`) and the seek (`_stopMode`/`_stopHeading`) can't cross-contaminate. RAW = `_applyBias`+`deadbandSnap`×2+`Move`; SYNC = `deadbandSnap`+`MoveSync`; GYRO = `deadbandSnap`+`MoveGyro`. Behavioral correctness of the extraction is bench-only — review the "no instance state" property carefully.
- **Reverse-GYRO corner:** an `ON`-forced GYRO seek in REVERSE issues `MoveGyro(-mag, heading)`. `@untested on hardware`; flag for the B3.7-reverse bench pass (common reverse seek is SYNC). Pass `heading = toImuYaw(_resolvedBodyHeading)` (already rear-facing).
- **No local C++ compiler:** every C++ AC (AC1-firmware, AC4, AC5, AC6, AC8, AC9, AC10) is CI-gated and cannot be run on the author's machine; rely on the ctest CI job.
- **Drive's 1-board reverse reject** stays — now unreachable from PathTracker (resolver degrades first) but guards the bare verb.
- **Rollback:** the change is additive behind the resolver; reverting T5's dispatch block + T6/T7's branches restores the prior (forward-only) behavior. `ACT_STOP_AT_LINE_REVERSE` is purely additive.
