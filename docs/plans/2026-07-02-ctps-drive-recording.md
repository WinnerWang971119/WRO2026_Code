# CTPS Drive/LineFollower Recording Fidelity Plan (issue #89)

**Goal:** Make the CTPS command-stream recorder faithfully capture the fluent Drive/LineFollower builder state it currently drops — the missing mock methods, the GYRO mode + held heading, distance-gated line-follow moves, and reverse direction — so the trajectory player renders authored missions in the right direction instead of the chassis's current facing.

**Approach:** The CTPS mock builders switch from recording at construction to recording once in their **destructor** (guarded by an `_executed` sentinel, exactly like the real builders), so chained setters (`.setMode(GYRO).setHeading(0).setReverse(true)`) are seen before the record is emitted. The captured `mode`/`heading`/`reverse` fields are carried additively through the same three-place `drive.method` contract (C++ recorder → command-stream JSON → Python loader whitelist → `track_builder` renderer). A new `lineFollowDist` drive method captures distance-gated LineFollower follows. A compile-time conformance TU pins the mock builder surface as a superset of the real one so this class of drift can't recur. Two pre-existing latent bugs found in the same code path (the `wallStop` `cm` serialization omission and the missing `LineFollowerEnableBuilder::flat()` mock) are fixed in the same PR.

## Scope

- **In scope:**
  - **Part 1 — mock surface completeness:** add `flat()` to the mock `DriveDistBuilder` + `DriveTimeBuilder`; add `setPreset(Config::SpeedPreset)` to all six mock Drive builders; add `flat()` to the mock `LineFollowerEnableBuilder`; `#include "Config.h"` in the mock `Drive.h` (matching the sibling `LineFollower.h` mock).
  - **Part 2 — mode/heading capture:** carry `mode` + `heading` through recorder → JSON → loader → renderer for `driveDist`, `stopAtLine`, `wallStop`. A GYRO move with an explicit heading renders **align-to-H then advance** (two keyframes). Heading is honored **only when `mode == "gyro"`** (mirrors firmware, which consults `setHeading` only under GYRO).
  - **LineFollower capture:** distance-gated follows — `followDistance(cm)` and `setEnabled(true).setDistanceTarget(cm)` — recorded as a new `drive` method `lineFollowDist`, rendered as a forward (or reverse) advance.
  - **Reverse direction:** capture a `reverse` bool for `lineFollowDist`, `stopAtLine`, and `wallStop`; render `chassis.reverse(cm)` when set.
  - **Recording seam:** all six Drive builders + the LineFollower enable-builder move from construction-time to `_executed`-guarded **destructor** recording.
  - **Bug fixes in the same path:** emit `cm` for `wallStop` in `commandStreamToJson` (today omitted → `float(None)` crash on replay); add the missing `LineFollowerEnableBuilder::flat()` mock.
  - **Regression guard:** a compile-time conformance TU that calls every real setter (with its real argument types) on each mock builder, so a missing/wrong-signature mock setter fails the build.
  - **Tests:** extend `ctps_dump --self-test` to assert the new recorder output; add Python loader + track_builder tests with new fixtures.
  - **Docs:** one ADR for the compile-time mock-conformance guard + DECISIONS.md index update.
- **Out of scope:**
  - Intersection-gated LineFollower follows (`followUntilIntersection` / `waitUntilIntersection`) — no fixed distance is known at record time without the field graph; documented as a limitation (renders as no advance, same as today).
  - `driveTime` geometric fidelity — stays encoded as the non-geometry `stop` method; no mode/heading. Documented, not fixed.
  - `turnDeg` / `turnToYaw` mode/heading — turns encode their heading change as the turn itself; unchanged.
  - Any `routes.h` / `routes.cpp` regeneration — no field-data change.
  - HTML player (`player_html.py` / `player.js`) — consumes Data Model B (Track keyframes/events), insulated from this Data Model A schema change; untouched.
  - Bumping `schema_version` — stays `1`, additive-optional.
  - Capturing the non-geometry profile/brake knobs (startSpeed/endSpeed/accel/brake/power) — CTPS playback is constant-velocity and ignores them.

## Decisions

- **Recording seam = destructor with `_executed` guard** — chosen over "record-then-patch-by-index" via `/debate` (Lean Yes, 7.25/10). Verification showed the real builders are *movable with an `_executed` sentinel* (`Drive.h:1499-1521`, `:1752-1758`), so destructor-emit guarded by `_executed` is a proven in-repo pattern that neutralizes the double-emit hazard. Mock stays non-movable (deleted copy/move) leaning on C++17 guaranteed elision; a `static_assert(!is_copy_constructible && !is_move_constructible)` documents that invariant; the `_executed` guard makes correctness robust even if that ever changes.
- **Schema stays `schema_version: 1`, additive-optional** — `mode`/`heading`/`reverse` are new optional keys; the loader defaults them to `None`. All 8 existing v1 fixtures load unchanged. A version bump would force the hard-reject loader to learn multi-version + a fixture rewrite for no gain.
- **`mode` = lowercase string (`"raw"|"sync"|"gyro"`); `heading` = number, emitted only when `setHeading` was called; `reverse` = bool** — matches the existing method-string serialization style; absence = "unset / current facing".
- **Renderer honors `heading` only when `mode == "gyro"`** — a `driveDist(35).setHeading(0)` with no `.setMode(GYRO)` is ignored on hardware, so the renderer must not drive along H in RAW/SYNC or it would diverge from the robot (Fable consult, corroborated by Codex).
- **GYRO+heading render = two-phase** — `turn_to_heading(H)` + a same-position heading keyframe, then advance + a translation keyframe. Literally renders "align to H, **then** advance" (chosen over one combined keyframe).
- **LineFollower follow encoded as `drive` op, method `lineFollowDist`** — reuses the existing 3-place `drive.method` contract rather than a new top-level op; smallest vocabulary change.
- **Superset guard = compile-time conformance TU** — a name-only Python regex checks names not signatures and already missed the `LineFollowerEnableBuilder::flat()` gap; a compile probe catches signature drift (e.g. `setMode(DriveMode)`→`setMode(uint8_t)`).
- **`wallStop` `cm` serialization is a real latent bug, fixed here** — `recordWallStop` stores `rec.cm` but `commandStreamToJson` only emits `cm` for `driveDist`/`stopAtLine`; a `wallStop` in a dumped trace reaches `float(None)` and crashes `track_builder`. Fixed by adding `wallStop` (and `lineFollowDist`) to the `cm`-emitting branch.
- **Recorder helper takes a small params struct, not a 5-positional-arg signature** — `ctps::recordDriveMove(method, DriveMoveParams{cm, mode, hasHeading, heading, reverse})` keeps the forward-geometry recorders readable (avoids a long positional arg list).
- **Loader validates `mode ∈ {raw,sync,gyro}` and `heading` numeric** — a malformed v1 field warns and is ignored rather than silently acquiring GYRO semantics (Codex consult).

## Acceptance Criteria

- [ ] `ctps_dump` compiles with a task/mission move chaining `.flat()` and/or `.setPreset(...)` on any Drive builder (Part 1 compile break removed). *(AC1)*
- [ ] A `driveDist(d).setMode(GYRO).setHeading(H)` records `method:"driveDist"`, `mode:"gyro"`, `heading:H` in the command-stream JSON; a plain `driveDist(d)` records neither `mode` (or `mode:"sync"`) nor `heading`. *(AC2)*
- [ ] `track_builder` renders a GYRO+heading `driveDist` as a rotation to H followed by an advance along H (two keyframes), ending translated along H facing H — not along the prior facing. *(AC3)*
- [ ] A non-GYRO `driveDist` (and any move without `mode:"gyro"`) still renders `forward(cm)` along current facing, byte-identical to today; existing track_builder TC1–TC5 pass unchanged. *(AC4)*
- [ ] `followDistance(cm)` and `setEnabled(true).setDistanceTarget(cm)` each record `method:"lineFollowDist"` with `cm`; `setEnabled(false)` and target-less `setEnabled(true)` record nothing. `track_builder` renders `lineFollowDist` as a forward advance. *(AC5)*
- [ ] A `.setReverse(true)` on `lineFollowDist` / `stopAtLine` / `wallStop` records `reverse:true` and renders `chassis.reverse(cm)` (backward along heading). *(AC6)*
- [ ] A `wallStop(cm)` command serializes with its `cm` and replays through `track_builder` without a `float(None)` crash. *(AC7 — regression)*
- [ ] The compile-time conformance TU fails to build if any mock Drive/LineFollower builder is missing a real setter or has a mismatched signature; it passes on the completed mocks. *(AC8)*
- [ ] `ctps_dump --self-test` asserts the recorded `mode`/`heading`/`reverse`/`lineFollowDist` fields and stays green in CI (`CTPS DUMP SELFTEST OK`). *(AC9)*
- [ ] The loader accepts `mode`/`heading`/`reverse`, whitelists `lineFollowDist`, and warns+ignores a malformed `mode`. All 8 existing v1 fixtures load unchanged. *(AC10)*
- [ ] Full `pytest tools/path_planner/tests/` and full `ctest` are green; no `routes.h`/`routes.cpp` change. *(AC11)*

## Data Model (command-stream JSON — Data Model A, schema_version 1)

A `drive` command entry gains three optional keys. Only the fields relevant to the method are emitted.

```jsonc
// driveDist / stopAtLine / wallStop:
{ "seq": 7, "op": "drive", "method": "driveDist",
  "cm": 35, "mode": "gyro", "heading": 0, "reverse": false }
// (mode omitted or "sync"/"raw" for non-gyro; heading omitted when setHeading never called;
//  reverse omitted/false when not set)

// lineFollowDist (LineFollower distance-gated follow):
{ "seq": 8, "op": "drive", "method": "lineFollowDist", "cm": 90, "reverse": false }

// turnDeg / turnToYaw (unchanged): { ..., "method": "turnDeg", "deg": -90 }
// stop / driveTime (unchanged, non-geometry): { ..., "method": "stop" }
```

### C++ `ctps::Record` additions (`ctps_record.h`)

```cpp
// added to struct Record (additive; defaults keep existing records byte-identical):
std::string mode;            // DRIVE: "raw"|"sync"|"gyro"; empty = not applicable (turns, stop, driveTime)
float       heading   = 0.0f;// DRIVE: held heading (gyro moves)
bool        hasHeading = false; // DRIVE: true iff setHeading() was called
bool        reverse   = false;  // DRIVE: reverse-direction flag (lineFollowDist/stopAtLine/wallStop)
```

### Python `Command` additions (`trace_loader.py`)

```python
mode:    Optional[str]   = None   # drive: "raw"|"sync"|"gyro"
heading: Optional[float] = None   # drive: held heading (gyro)
reverse: Optional[bool]  = None   # drive: reverse-direction flag
```

## Contracts & Interfaces

Single source of truth for every seam more than one task touches.

### The three-place `drive.method` contract

`lineFollowDist` (and the `mode`/`heading`/`reverse` payload) must appear identically in all three places or commands are silently dropped/mis-rendered:

| Place | File | Owner | What changes |
|-------|------|-------|--------------|
| Recorder emit | `WRO2026/test/ctps_mocks/ctps_record.cpp` (`commandStreamToJson`) | T1 | emit `mode`/`heading`/`reverse`; emit `cm` for `wallStop` **and** `lineFollowDist` |
| Loader whitelist | `tools/path_planner/ctps/trace_loader.py` (`_KNOWN_DRIVE_METHODS`, `Command`) | T6 | add `"lineFollowDist"`; add `mode`/`heading`/`reverse` fields; validate mode/heading |
| Renderer dispatch | `tools/path_planner/ctps/track_builder.py` (`build`, `_drive_label`) | T7 | `lineFollowDist` geometry branch; gyro two-phase; reverse; label |

### Shared types / signatures (owner → consumers)

- **`struct ctps::Record` fields** (`ctps_record.h`) — owner **T1**; consumers **T2** (Drive mock), **T3** (LineFollower mock), **T4** (self-test).
- **`ctps::DriveMoveParams { float cm; const char* mode; bool hasHeading; float heading; bool reverse; }`** + **`void ctps::recordDriveMove(const char* method, const DriveMoveParams&)`** — owner **T1**; consumers **T2**, **T3**. Replaces/extends `recordDriveDist`/`recordStopAtLine`/`recordWallStop`; adds the `lineFollowDist` producer path.
- **JSON key set** `{mode:"raw"|"sync"|"gyro", heading:number, reverse:bool}` and method string `"lineFollowDist"` — owner **T1** (emit) / **T6** (parse); consumers **T7** (render), **T8/T9** (fixtures).
- **`Command.mode/heading/reverse`** (`trace_loader.py`) — owner **T6**; consumer **T7**.
- **Mock `#include "Config.h"`** in `ctps_mocks/Drive.h` for `Config::SpeedPreset` — owner **T2** (pattern already in `ctps_mocks/LineFollower.h`).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/test/ctps_mocks/ctps_record.h` / `.cpp` | T1 | T2, T3, T4 |
| `WRO2026/test/ctps_mocks/Drive.h` | T2 | T4, T5 |
| `WRO2026/test/ctps_mocks/LineFollower.h` | T3 | T4, T5 |
| `WRO2026/test/ctps_dump.cpp` | T4 | — |
| `WRO2026/test/CMakeLists.txt` + new conformance TU | T5 | — |
| `tools/path_planner/ctps/trace_loader.py` | T6 | T7, T8 |
| `tools/path_planner/ctps/track_builder.py` | T7 | T9 |
| `tools/path_planner/tests/test_ctps_trace_loader.py` (+ fixtures) | T8 | — |
| `tools/path_planner/tests/test_ctps_track_builder.py` (+ fixtures) | T9 | — |
| `docs/decisions/0NN-*.md` + `DECISIONS.md` | T10 | — |

### Naming (exact strings that must match across tasks)

- Drive method string: `"lineFollowDist"` (recorder, loader whitelist, renderer, label — byte-identical).
- Mode strings: `"raw"`, `"sync"`, `"gyro"` (recorder emit ↔ loader validation set ↔ renderer `== "gyro"` gate).
- JSON keys: `"mode"`, `"heading"`, `"reverse"`, `"cm"`.

## Error Handling

- **Malformed `mode`** (not in `{raw,sync,gyro}`): loader emits a `UserWarning` and treats mode as unset (renders current facing) rather than raising or silently applying GYRO.
- **`heading` present but non-numeric:** loader warns and drops the heading.
- **`mode:"gyro"` with no `heading`:** renders along current facing (bare GYRO holds current `_heldYaw` on hardware).
- **`wallStop` / `lineFollowDist` with missing `cm`** (legacy/partial trace): loader leaves `cm=None`; renderer guards `cm is None` → narrate-only, no geometry (no `float(None)` crash). This is the defensive fix for the AC7 regression.
- **A move builder ever added to a mock:** the `_executed` guard makes the destructor idempotent (no double-emit); the `static_assert(!copy && !move)` turns it into a compile error first.
- **Unknown drive method** (forward-compat): existing loader behavior — warn + skip — unchanged.

## Testing Strategy

**Levels:** Unit (C++ self-test assertions; Python loader), Integration (Python track_builder end-to-end geometry), Compile (conformance TU).

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `ctps_dump --self-test`: GYRO+heading driveDist in the synthetic segment | Unit (C++) | Record has `method:"driveDist"`, `mode=="gyro"`, `hasHeading`, `heading==0` |
| TC2 | Self-test: `lineFollowDist` + `reverse` capture | Unit (C++) | A `lineFollowDist` record with `cm` present; a `reverse==true` record on a reversed move |
| TC3 | Conformance TU: every real Drive/LF setter called on each mock builder | Compile | Builds clean on completed mocks; fails if a setter is missing/mismatched |
| TC4 | Conformance TU with a deliberately-removed mock setter (local check, not committed) | Compile | Build fails — proves the guard bites |
| TC5 | Loader parses `mode`/`heading`/`reverse` + whitelists `lineFollowDist` | Unit (Py) | `Command` fields populated; `lineFollowDist` not skipped |
| TC6 | Loader with malformed `mode:"turbo"` | Unit (Py) | `UserWarning`, mode treated as unset, command still loads |
| TC7 | All 8 existing v1 fixtures load unchanged | Unit (Py) | No new required keys; `mode/heading/reverse` default `None` |
| TC8 | track_builder: `driveDist mode:"gyro" heading:90` from a chassis facing 0 | Integration | Two keyframes; chassis ends displaced along +y, facing 90 |
| TC9 | track_builder: `driveDist heading:90` with `mode:"sync"` (no gyro) | Integration | Renders along current facing (heading ignored) — firmware parity |
| TC10 | track_builder: `lineFollowDist 90` advances chassis 90 cm forward | Integration | One geometry keyframe; drive-type event emitted |
| TC11 | track_builder: `lineFollowDist`/`stopAtLine` with `reverse:true` | Integration | Chassis moves backward along heading (`chassis.reverse`) |
| TC12 | track_builder: `wallStop 20` replays without crashing | Integration | Forward advance ~20 cm; no `float(None)` (AC7 regression guard) |

**Test data:** hand-authored Data Model A fixtures under `tools/path_planner/tests/fixtures/ctps_*.json` (new: gyro+heading, lineFollowDist, reverse, wallStop). Every `goto` target reachable from the toy field `START_NODE`. New fixture method names must not collide with the unknown-method fixture's `strafeLeft`/`teleport`. C++ self-test extends the existing `recordManipDriveSegment` synthetic segment.

**Run commands:** `uv run --extra dev pytest tools/path_planner/tests/`; `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Command-stream schema + serializer + wallStop fix | — | med | `WRO2026/test/ctps_mocks/ctps_record.h`, `ctps_record.cpp` | Add `mode`/`heading`/`hasHeading`/`reverse` to `struct Record`. Add `ctps::DriveMoveParams` + `recordDriveMove(method, params)` producing DRIVE records; keep `recordTurnDeg/turnToYaw/recordDriveStop`. In `commandStreamToJson` DRIVE case: emit `cm` for `driveDist`/`stopAtLine`/**`wallStop`**/**`lineFollowDist`**; emit `mode` when non-empty, `heading` when `hasHeading`, `reverse` when true. Update ctps_record.h doxygen (mode/heading/reverse; lineFollowDist producer). Satisfies AC2, AC7. Contracts: owns Record + recordDriveMove + JSON key set. |
| T2 | Drive mock: destructor recording + missing setters + capture | T1 | high | `WRO2026/test/ctps_mocks/Drive.h` | `#include "Config.h"`. Convert all six builders to store members + emit once in an `_executed`-guarded destructor (delete copy+move as today; add `static_assert(!is_copy_constructible && !is_move_constructible)` per recording builder). Add `flat()` to DriveDist+DriveTime; `setPreset(Config::SpeedPreset)` to all six. Capture `mode`/`heading`(+`hasHeading`)/`reverse` on driveDist/stopAtLine/wallStop and emit via `recordDriveMove`. driveTime still emits `stop`; turnDeg/turnToYaw unchanged (no mode/heading). Rewrite the file docstring (construction→destructor seam rationale). Satisfies AC1, AC2, AC6. |
| T3 | LineFollower mock: flat() + lineFollowDist capture | T1 | med | `WRO2026/test/ctps_mocks/LineFollower.h` | Add `flat()` to `LineFollowerEnableBuilder`. Make the enable-builder store `_enabled`/`_distanceTarget`/`_hasTarget`/`_reverse` and emit `recordDriveMove("lineFollowDist", ...)` in an `_executed`-guarded destructor **only when** `_enabled && _hasTarget`. Make `followDistance(float cm)` record `lineFollowDist` directly (add `reverse` from `getReverse()` if applicable, else false). `setEnabled(false)` / target-less enable record nothing. Satisfies AC5, AC6. |
| T4 | Extend ctps_dump self-test | T2, T3 | low | `WRO2026/test/ctps_dump.cpp` | Extend `recordManipDriveSegment` with a `driveDist(..).setMode(GYRO).setHeading(0)`, a `lineFollowDist` follow, and a reversed move. Add assertions on the recorded `mode`/`hasHeading`/`heading`/`reverse` and the `lineFollowDist` method. Keep the existing ordered-op walk green. Update the "record at construction" comment to "record at destructor". Satisfies AC9. |
| T5 | Compile-time mock-conformance TU | T2, T3 | med | `WRO2026/test/ctps_mocks/test_ctps_mock_conformance.cpp` (new), `WRO2026/test/CMakeLists.txt` | New host TU that, on a discarded temporary of each mock builder, chains **every** real setter with its real arg type (`.setPreset(Config::SpeedPreset::NORMAL).flat().setMode(DriveMode::GYRO).setHeading(0).setReverse(true)...`), for all six Drive builders + `LineFollowerEnableBuilder`. Add a `ctps_mock_conformance` target using the existing `add_ctps_mock_force_includes` + fixture-routes wiring; register as a ctest (build success = pass). Do NOT put the TU in a dir on other targets' include path. Satisfies AC8. |
| T6 | Loader: fields + whitelist + validation | — | med | `tools/path_planner/ctps/trace_loader.py` | Add `mode`/`heading`/`reverse` to `Command` and to the named-keys set. Add `"lineFollowDist"` to `_KNOWN_DRIVE_METHODS`. Validate `mode ∈ {raw,sync,gyro}` (warn+unset otherwise) and `heading` numeric (warn+drop otherwise). Existing unknown-method + unknown-op behavior unchanged. Satisfies AC10 (works against the T1 JSON contract, no code dependency). |
| T7 | Renderer: lineFollowDist + gyro two-phase + reverse | T6 | high | `tools/path_planner/ctps/track_builder.py` | In the `drive` branch: `lineFollowDist` joins the forward-geometry set. For `driveDist`/`stopAtLine`/`wallStop`/`lineFollowDist`: if `reverse` → `chassis.reverse(cm)` else `chassis.forward(cm)`; guard `cm is None` (narrate-only). For `mode == "gyro"` **and** heading present on driveDist/stopAtLine/wallStop: `turn_to_heading(H)` + heading keyframe, then advance + translation keyframe (two-phase). Non-gyro/no-heading path unchanged. Extend `_drive_label` with `lineFollowDist(cm)` and append `mode`/`heading` only when present (do NOT reformat existing method labels). Satisfies AC3, AC4, AC6. |
| T8 | Loader tests + fixtures | T6 | low | `tools/path_planner/tests/test_ctps_trace_loader.py`, `tools/path_planner/tests/fixtures/ctps_gyro_heading.json` (new, + reverse/lineFollowDist fixtures) | TC5–TC7: parse mode/heading/reverse; whitelist lineFollowDist; malformed mode warns+unsets; all 8 existing fixtures still load. Fixture method names avoid `strafeLeft`/`teleport`. |
| T9 | Renderer tests + fixtures | T7 | med | `tools/path_planner/tests/test_ctps_track_builder.py` (+ fixtures) | TC8–TC12: gyro two-phase geometry (ends along H facing H); non-gyro heading ignored; lineFollowDist advance; reverse moves backward; wallStop replays without crash. Existing TC1–TC5 must stay green (non-gyro path unchanged). |
| T10 | ADR for compile-time mock-conformance guard | T5 | low | `docs/decisions/0NN-ctps-mock-conformance-guard.md` (next free number — verify highest in `docs/decisions/`), `DECISIONS.md` | Short ADR: "the CTPS mock builder surface is a compile-verified superset of the real builder surface (conformance TU), and the mock records at destructor with an `_executed` guard mirroring the real builders." Note it extends ADR-030/049 (host-mock trust boundary) and the ADR-033 CTPS trace. Update the DECISIONS.md index + status table. Verify no ADR-number collision before assigning. |

**Parallelism:** T1 and T6 start immediately (T6 codes against the JSON contract in this spec, not T1's code). After T1: T2 ∥ T3. After T2+T3: T4 ∥ T5. After T6: T7. After T7: T9. After T6: T8. After T5: T10. Two file-family lanes (C++: T1→{T2,T3}→{T4,T5}→T10; Python: T6→T7→T9, T6→T8) join only at CI.

## Notes for Implementer

- **The `_executed` guard is the load-bearing detail.** Copy the real builders' pattern (`Drive.h:1499-1521`, `:1752-1758`): destructor does `if (_executed) return; _executed = true;` then emits. Keep the mock non-movable (don't add a move ctor); the `static_assert` documents that, and guaranteed elision means no move is invoked on the by-value return. Result: exactly-one emit, robust even if movability is ever reintroduced.
- **Emission order:** for the canonical one-verb-per-statement discarded-temporary shape, destructor-emit lands at the statement's `;` in authored order — identical to construction order. Do not author two Drive verbs in a single full-expression (temporaries destruct in reverse construction order); no existing or authored code does this.
- **Do not model firmware default-resolution in the mock** (the epic #107 "behavioral mock" trap): the mock captures only *explicit* setter calls. A `wallStop` with no explicit `setMode` records mode-absent even though firmware defaults it to GYRO — that is the correct thin-mock behavior; the render stays a nominal forward push.
- **`_drive_label` regression risk:** existing track_builder tests assert manip labels (TC2) and drive-event presence (TC5), not exact drive labels — but keep existing method label formatting byte-identical and only *append* mode/heading conditionally, to be safe.
- **The `cm is None` guard in track_builder is mandatory** — it is the actual fix for the wallStop crash (AC7). Even after T1 emits `cm` for wallStop, keep the guard for partial/legacy traces.
- **Conformance TU wiring gotchas:** put the TU in its own file force-included/compiled only into its own target; `ctps_mocks/` is already the CTPS include root. Reuse `add_ctps_mock_force_includes` + `add_fixture_routes_force_include`. A missing setter should fail *this* target's build, surfaced by ctest.
- **Run the FULL pytest and FULL ctest**, not per-file — the schema/contract touches both suites and the C++ self-test.
- **Rollback:** the change is additive (schema, new method, new mock members). Reverting T7/T6 restores current-facing rendering; reverting T2/T3 restores construction recording. No persisted data or field-data migration is involved.
