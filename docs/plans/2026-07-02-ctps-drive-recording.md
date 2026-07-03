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
- **`mode` = lowercase string (`"raw"|"sync"|"gyro"`), emitted only when `setMode` was explicitly called; `heading` = number, emitted only when `setHeading` was called; `reverse` = JSON bool, emitted only when `setReverse(true)`** — matches the existing method-string serialization style; an absent `mode` means "unset → render current facing" (the mock captures explicit setter calls only, never firmware default-resolution). The loader parses `reverse` as a JSON boolean, never via Python truthiness (the string `"false"` is truthy, so a naive `bool(x)` would invert intent).
- **Renderer honors `heading` only when `mode == "gyro"`** — a `driveDist(35).setHeading(0)` with no `.setMode(GYRO)` is ignored on hardware, so the renderer must not drive along H in RAW/SYNC or it would diverge from the robot (Fable consult, corroborated by Codex).
- **GYRO+heading render = two-phase** — `turn_to_heading(H)` + a same-position heading keyframe, then advance + a translation keyframe. Literally renders "align to H, **then** advance" (chosen over one combined keyframe).
- **LineFollower follow encoded as `drive` op, method `lineFollowDist`** — reuses the existing 3-place `drive.method` contract rather than a new top-level op; smallest vocabulary change.
- **Superset guard = compile-time conformance TU ("enumerated compile probe") + a Python name-superset check** (two complementary mechanisms). The TU chains every *current* real setter (with its real arg type) on each mock builder: it catches a removed/renamed mock setter and a signature-incompatible existing setter, but does NOT auto-catch a real builder *gaining* a new setter (it stays green until a human adds the call — the same manual step as updating the mock). A libcst/regex name-set check over both headers (mock fluent-builder method-set ⊇ real) catches that real-grew case — the actual #89 drift class. Together they deliver "can't drift again." The #89 `flat()`/`setPreset()` gap arose because the mock was never updated when the real builders grew those setters — **not** because a checker "missed" it (no name-only checker exists in the repo today). Known TU limit: it verifies call-site *assignability*, not exact parameter types (a convertible type like `float` vs `double`/`int` can pass); acceptable for a mock that only records commanded values.
- **`wallStop` `cm` serialization is a real latent bug, fixed here** — `recordWallStop` stores `rec.cm` but `commandStreamToJson` only emits `cm` for `driveDist`/`stopAtLine`; a `wallStop` in a dumped trace reaches `float(None)` and crashes `track_builder`. Fixed by adding `wallStop` (and `lineFollowDist`) to the `cm`-emitting branch.
- **Recorder helper takes a small params struct, not a 5-positional-arg signature** — `ctps::recordDriveMove(method, DriveMoveParams{cm, mode, hasHeading, heading, reverse})` keeps the forward-geometry recorders readable (avoids a long positional arg list).
- **Loader validates `mode ∈ {raw,sync,gyro}` and `heading` numeric** — a malformed v1 field warns and is ignored rather than silently acquiring GYRO semantics (Codex consult). A non-string/non-list `mode` is type-checked before set-membership so it warns rather than raising.
- **The serializer output is verified at two non-substitutable tiers** — (a) the C++ self-test calls `commandStreamToJson` and substring-asserts the new keys (cheap, early, in the host-tests job); (b) a dedicated CI-gated end-to-end test runs the real `ctps_dump --fixture` through `trace_loader` + `track_builder` (the only thing that verifies Python *consumption* of real serializer output, incl. the `wallStop`-no-crash AC). Neither replaces the other (critic + judge ruling).
- **GYRO+heading composes with reverse** — when a `driveDist`/`stopAtLine`/`wallStop` is `mode:"gyro"` with a heading AND `reverse:true`, the two-phase render is `turn_to_heading(H)` then `chassis.reverse(cm)` (align to H, then back up along H). The advance keyframe honors `reverse` in both the gyro and non-gyro branches.

## Acceptance Criteria

- [ ] `ctps_dump` compiles with a task/mission move chaining `.flat()` and/or `.setPreset(...)` on any Drive builder (Part 1 compile break removed). *(AC1)*
- [ ] A `driveDist(d).setMode(GYRO).setHeading(H)` records `method:"driveDist"`, `mode:"gyro"`, `heading:H` in the command-stream JSON; a plain `driveDist(d)` (no explicit `setMode`/`setHeading`) records neither `mode` nor `heading`. *(AC2)*
- [ ] `track_builder` renders a GYRO+heading `driveDist` as a rotation to H followed by an advance along H (two keyframes), ending translated along H facing H — not along the prior facing. *(AC3)*
- [ ] A non-GYRO `driveDist` (and any move without `mode:"gyro"`) still renders `forward(cm)` along current facing, byte-identical to today; existing track_builder TC1–TC5 pass unchanged. *(AC4)*
- [ ] `followDistance(cm)` and `setEnabled(true).setDistanceTarget(cm)` each record `method:"lineFollowDist"` with `cm`; `setEnabled(false)` and target-less `setEnabled(true)` record nothing. `track_builder` renders `lineFollowDist` as a forward advance. *(AC5)*
- [ ] A `.setReverse(true)` on `lineFollowDist` / `stopAtLine` / `wallStop` records `reverse:true` and renders `chassis.reverse(cm)` (backward along heading). *(AC6)*
- [ ] A `wallStop(cm)` command serializes with its `cm`; feeding real `ctps_dump --fixture` output through `track_builder` (end-to-end, AC13) replays it without a `float(None)` crash, and a cm-less `wallStop`/`lineFollowDist` fixture hits the `cm is None` narrate-only guard. *(AC7 — regression; verified by AC13 + the negative-path fixture, not the happy-path TC)*
- [ ] The compile-time conformance TU fails to build if any mock Drive/LineFollower builder drops/renames a setter the TU exercises, or makes an exercised setter's signature incompatible with the real call. The Python name-superset check fails if any real fluent-builder method name is absent from the corresponding mock (the real-grew #89 case). Both pass on the completed mocks. *(AC8)*
- [ ] The C++ self-test calls `commandStreamToJson` and substring-asserts the serialized `mode`/`heading`/`reverse` + the `wallStop`/`lineFollowDist` `cm` keys. *(AC12)*
- [ ] A dedicated CI-gated test builds+runs `ctps_dump --fixture`, pipes it through `trace_loader` + `track_builder`, and asserts the new fields survive and a `wallStop` replays without crashing. *(AC13)*
- [ ] `ctps_dump --self-test` asserts the recorded `mode`/`heading`/`reverse`/`lineFollowDist` fields and stays green in CI (`CTPS DUMP SELFTEST OK`). *(AC9)*
- [ ] The loader accepts `mode`/`heading`/`reverse`, whitelists `lineFollowDist`, and warns+ignores a malformed `mode`. All 8 existing v1 fixtures load unchanged. *(AC10)*
- [ ] Full `pytest tools/path_planner/tests/` and full `ctest` are green; no `routes.h`/`routes.cpp` change. *(AC11)*

## Data Model (command-stream JSON — Data Model A, schema_version 1)

A `drive` command entry gains three optional keys. Only the fields relevant to the method are emitted.

```jsonc
// driveDist / stopAtLine / wallStop with GYRO + explicit heading:
{ "seq": 7, "op": "drive", "method": "driveDist", "cm": 35, "mode": "gyro", "heading": 0 }
// plain driveDist (no setMode/setHeading/setReverse): none of the new keys appear:
{ "seq": 7, "op": "drive", "method": "driveDist", "cm": 35 }
// Each of mode / heading / reverse is emitted ONLY when its setter was called
// (mode only on explicit setMode; heading only on setHeading; reverse only on setReverse(true)).
// An absent mode → the renderer uses the current facing.

// lineFollowDist (LineFollower distance-gated follow), reversed:
{ "seq": 8, "op": "drive", "method": "lineFollowDist", "cm": 90, "reverse": true }

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
| `tools/path_planner/tests/test_ctps_trace_loader.py` + `tests/fixtures/ctps_*.json` (new) | T8 | T9 (consumes fixtures) |
| `tools/path_planner/tests/test_ctps_track_builder.py` | T9 | — |
| `tools/path_planner/tests/test_ctps_end_to_end.py` (new) + CI wiring | T11 | — |
| `tools/path_planner/tests/test_ctps_mock_superset.py` (new) | T12 | — |
| `docs/decisions/0NN-*.md` + `DECISIONS.md` | T10 | — |

### Naming (exact strings that must match across tasks)

- Drive method string: `"lineFollowDist"` (recorder, loader whitelist, renderer, label — byte-identical).
- Mode strings: `"raw"`, `"sync"`, `"gyro"` (recorder emit ↔ loader validation set ↔ renderer `== "gyro"` gate).
- JSON keys: `"mode"`, `"heading"`, `"reverse"`, `"cm"`.

## Error Handling

- **Malformed `mode`** (not a string, or a string not in `{raw,sync,gyro}`): loader type-checks it is a string *before* the set-membership test, emits a `UserWarning`, and treats mode as unset (renders current facing) rather than raising or silently applying GYRO.
- **`heading` present but non-numeric:** loader warns and drops the heading.
- **`reverse` parsing:** loader accepts only a JSON boolean; a non-bool value (e.g. the string `"false"`, which is truthy under `bool()`) warns and is treated as `False`. Never `bool(raw.get("reverse"))`.
- **`mode:"gyro"` with no `heading`:** renders along current facing (bare GYRO holds current `_heldYaw` on hardware).
- **`wallStop` / `lineFollowDist` with missing `cm`** (legacy/partial trace): loader leaves `cm=None`; renderer guards `cm is None` → narrate-only, no geometry (no `float(None)` crash). This is the defensive fix for the AC7 regression.
- **A move builder ever added to a mock:** the `_executed` guard makes the destructor idempotent (no double-emit); the `static_assert(!copy && !move)` turns it into a compile error first.
- **Unknown drive method** (forward-compat): existing loader behavior — warn + skip — unchanged.

## Testing Strategy

**Levels:** Unit (C++ self-test; Python loader), Integration (Python track_builder geometry + a real-binary end-to-end seam test), Compile (conformance TU).

> **TC-ID note:** new cases use the `NC*` prefix to avoid collision with the *existing* `test_ctps_track_builder.py` TC1–TC5 and the `ctps_dump.cpp` TC8/TC9, which keep their meanings.

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| NC1 | Self-test: GYRO+heading driveDist in the synthetic segment | Unit (C++) | `Record` has `method:"driveDist"`, `mode=="gyro"`, `hasHeading`, `heading==0` |
| NC2 | Self-test: `lineFollowDist` from BOTH producers (`followDistance` + `setEnabled(true).setDistanceTarget()`); `setEnabled(false)` and target-less `setEnabled(true)` record NOTHING; a `reverse==true` record | Unit (C++) | Both producers emit one `lineFollowDist` each; the two silence cases emit none; reverse captured (AC5, AC6) |
| NC3 | Self-test: `commandStreamToJson` substring assertions | Unit (C++) | JSON contains `"mode": "gyro"`, `"heading"`, `"reverse"`, a `"cm"` on the `wallStop` record and on `lineFollowDist`; a plain `driveDist` emits no `"mode"`/`"heading"` (AC12) |
| NC4 | Conformance TU (always-built): every real setter chained on each mock builder with its **real arg type** (turnDeg/turnToYaw use `setMode(uint8_t)`, not `DriveMode`) | Compile | Builds+links+runs (returns 0); a missing/renamed/signature-incompatible mock setter breaks the build (AC8) |
| NC5 | Python name-superset check: positive (current mocks ⊇ real) + negative (synthetic mock missing a method) | Unit (Py) | Positive passes; negative raises/asserts — proves the real-grew guard bites (AC8) |
| NC6 | Loader parses `mode`/`heading`/`reverse`; whitelists `lineFollowDist` | Unit (Py) | `Command` fields populated; `lineFollowDist` not skipped (AC10) |
| NC7 | Loader negative paths: non-string & unknown-string `mode`; non-numeric `heading`; `reverse:"false"` | Unit (Py) | Each warns; mode→unset, heading→dropped, `reverse` string → `False` (not truthy); command still loads |
| NC8 | All 8 existing v1 fixtures load unchanged | Unit (Py) | `mode/heading/reverse` default `None`; no new required keys (AC7-compat) |
| NC9 | track_builder: `driveDist mode:"gyro" heading:90` from a chassis facing 0 | Integration | Exactly 3 keyframes (seed → rotation at unchanged position → translation at heading 90), monotonic timestamps; ends displaced along +y facing 90 (AC3) |
| NC10 | track_builder: `driveDist heading:90` with `mode:"sync"` (no gyro) | Integration | Renders along current facing, heading ignored — firmware parity (AC4) |
| NC11 | track_builder: `lineFollowDist 90` | Integration | One geometry keyframe, +90 cm forward; drive-type event emitted (AC5) |
| NC12 | track_builder: `reverse:true` on `lineFollowDist` / `stopAtLine` / `wallStop` | Integration | `chassis.reverse` — backward along heading (AC6) |
| NC13 | track_builder: `stopAtLine mode:"gyro" heading:H reverse:true` (gyro∧reverse composition); plus a GYRO `stopAtLine`/`wallStop` (not just driveDist) | Integration | `turn_to_heading(H)` then `chassis.reverse(cm)`; GYRO honored on all three forward-geometry methods (AC3, AC6) |
| NC14 | track_builder: cm-less `wallStop`/`lineFollowDist` fixture (legacy/partial trace) | Integration | `cm is None` → narrate-only, no geometry, no `float(None)` crash (AC7 negative path) |
| NC15 | End-to-end: build+run `ctps_dump --fixture` → `trace_loader` → `track_builder` | Integration (gated) | New fields survive the real serializer; a `wallStop` replays without crashing (AC13) |
| NC16 | Regression: existing `test_ctps_track_builder` TC1–TC5 + the `ctps_dump` ordered-op self-test walk | Regression | Green — non-gyro/no-heading path byte-identical (AC4, AC11) |

**Test data:** hand-authored Data Model A fixtures under `tools/path_planner/tests/fixtures/ctps_*.json` — **owned solely by T8** (T9 consumes them; T9 is blocked on T8): `ctps_gyro_heading.json`, `ctps_line_follow.json`, `ctps_reverse.json`, `ctps_wallstop_no_cm.json`. Every `goto` target reachable from the toy field `START_NODE`. New fixture method names must not collide with the unknown-method fixture's `strafeLeft`/`teleport`. The C++ self-test extends the existing `recordManipDriveSegment` synthetic segment. The NC15 end-to-end test is CI-gated: `--ignore` it in the broad pytest job and invoke it by explicit path in a dedicated job that first builds `ctps_dump` (else it goes red when the binary is absent — see [[gotcha-ci-gated-test-broad-pytest-collection]]).

**Run commands:** `uv run --extra dev pytest tools/path_planner/tests/`; `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Command-stream schema + serializer + wallStop fix | — | med | `WRO2026/test/ctps_mocks/ctps_record.h`, `ctps_record.cpp` | Add `mode`/`heading`/`hasHeading`/`reverse` to `struct Record`. Add `ctps::DriveMoveParams` + `recordDriveMove(method, params)` producing DRIVE records; keep `recordTurnDeg/turnToYaw/recordDriveStop`. In `commandStreamToJson` DRIVE case: emit `cm` for `driveDist`/`stopAtLine`/**`wallStop`**/**`lineFollowDist`**; emit `mode` when non-empty, `heading` when `hasHeading`, `reverse` when true. Update ctps_record.h doxygen (mode/heading/reverse; lineFollowDist producer). Satisfies AC2, AC7. Contracts: owns Record + recordDriveMove + JSON key set. |
| T2 | Drive mock: destructor recording + missing setters + capture | T1 | high | `WRO2026/test/ctps_mocks/Drive.h` | `#include "Config.h"`. Convert all six builders to store members + emit once in an `_executed`-guarded destructor (delete copy+move as today; add `static_assert(!is_copy_constructible && !is_move_constructible)` per recording builder). Add `flat()` to DriveDist+DriveTime; `setPreset(Config::SpeedPreset)` to all six. Capture `mode`/`heading`(+`hasHeading`)/`reverse` on driveDist/stopAtLine/wallStop and emit via `recordDriveMove`. driveTime still emits `stop`; turnDeg/turnToYaw unchanged (no mode/heading). Rewrite the file docstring (construction→destructor seam rationale). Satisfies AC1, AC2, AC6. |
| T3 | LineFollower mock: flat() + lineFollowDist capture | T1 | med | `WRO2026/test/ctps_mocks/LineFollower.h` | Add `flat()` to `LineFollowerEnableBuilder`. Make the enable-builder store `_enabled`/`_distanceTarget`/`_hasTarget`/`_reverse` and emit `recordDriveMove("lineFollowDist", ...)` in an `_executed`-guarded destructor **only when** `_enabled && _hasTarget`. `followDistance(float cm)` records `lineFollowDist` **directly** — it must **not** delegate to `setEnabled().setDistanceTarget()` (the real one does; delegating here would double-record via the enable-builder destructor). For `followDistance` reverse capture, add a persistent facade-level `_reverse` member set by `setReverse(bool)` and read at record time (the current mock `getReverse()` hard-returns `false`). `setEnabled(false)` / target-less enable record nothing. Satisfies AC5, AC6. |
| T4 | Extend ctps_dump self-test (records + serialized JSON) | T2, T3 | med | `WRO2026/test/ctps_dump.cpp` | Extend `recordManipDriveSegment` with a `driveDist(..).setMode(GYRO).setHeading(0)`, `lineFollowDist` from **both** producers, the two silence cases, and a reversed move. Assert the in-memory `Record` fields (`mode`/`hasHeading`/`heading`/`reverse`/`lineFollowDist`). **Also** call `ctps::commandStreamToJson(...)` and substring-assert the serialized keys (`"mode": "gyro"`, `"heading"`, `"reverse"`, `wallStop`/`lineFollowDist` `"cm"`; plain `driveDist` has no `"mode"`/`"heading"`). Keep the ordered-op walk green. Update the "record at construction" comment to "record at destructor". Satisfies AC9, AC12; NC1–NC3. |
| T5 | Compile-time mock-conformance TU | T2, T3 | med | `WRO2026/test/ctps_mocks/test_ctps_mock_conformance.cpp` (new), `WRO2026/test/CMakeLists.txt` | New host TU with a `main(){return 0;}` that, on a discarded temporary of each mock builder, chains **every** real setter with its **real arg type** — note `turnDeg`/`turnToYaw` take `setMode(uint8_t)` (NOT `DriveMode`), the other four take `setMode(DriveMode)`; `.setPreset(Config::SpeedPreset::NORMAL)`, `.flat()` only where real, etc. — for all six Drive builders + `LineFollowerEnableBuilder`. The builder destructors call `ctps::recordDriveMove`, so the target must **link `ctps_record.cpp` + `fixtures/routes_fixture.cpp`** and use `add_ctps_mock_force_includes` + `add_fixture_routes_force_include`; it compiles+links+runs. Always-built target registered as a ctest; a missing/renamed/mismatched mock setter breaks the build. TU only verifies call-site assignability, not exact param types (documented limit). Do NOT put the TU in a dir on other targets' include path. Satisfies AC8 (signature/removed-drift half). |
| T6 | Loader: fields + whitelist + validation | — | med | `tools/path_planner/ctps/trace_loader.py` | Add `mode`/`heading`/`reverse` to `Command` and to the named-keys set. Add `"lineFollowDist"` to `_KNOWN_DRIVE_METHODS`. Validate: `mode` is a string in `{raw,sync,gyro}` (type-check before membership; warn+unset otherwise); `heading` numeric (warn+drop); `reverse` a JSON bool (warn+`False` otherwise — never `bool("false")`). Existing unknown-method + unknown-op behavior unchanged. Fix the stale `Command.fields` doc (see Notes). Satisfies AC10 (works against the T1 JSON contract, no code dependency). |
| T7 | Renderer: lineFollowDist + gyro two-phase + reverse | T6 | high | `tools/path_planner/ctps/track_builder.py` | In the `drive` branch: `lineFollowDist` joins the forward-geometry set. For `driveDist`/`stopAtLine`/`wallStop`/`lineFollowDist`: if `reverse` → `chassis.reverse(cm)` else `chassis.forward(cm)`; guard `cm is None` (narrate-only). For `mode == "gyro"` **and** heading present on driveDist/stopAtLine/wallStop: `turn_to_heading(H)` + heading keyframe, then advance + translation keyframe (two-phase). Non-gyro/no-heading path unchanged. Extend `_drive_label` with `lineFollowDist(cm)` and append `mode`/`heading` only when present (do NOT reformat existing method labels). Satisfies AC3, AC4, AC6. |
| T8 | Loader tests + **sole fixture owner** | T6 | low | `tools/path_planner/tests/test_ctps_trace_loader.py`, `tools/path_planner/tests/fixtures/ctps_gyro_heading.json` + `ctps_line_follow.json` + `ctps_reverse.json` + `ctps_wallstop_no_cm.json` (all new) | Authors ALL new fixtures (T9 consumes them). NC6–NC8: parse mode/heading/reverse; whitelist lineFollowDist; negative paths (non-string/unknown mode, non-numeric heading, `reverse:"false"`); all 8 existing fixtures still load. Fixture method names avoid `strafeLeft`/`teleport`. |
| T9 | Renderer tests | T7, T8 | med | `tools/path_planner/tests/test_ctps_track_builder.py` | NC9–NC14, NC16: gyro two-phase (exactly 3 keyframes; ends along H facing H); non-gyro heading ignored; lineFollowDist advance; reverse backward; gyro∧reverse composition; cm-less narrate-only (no crash); existing TC1–TC5 stay green. Consumes T8's fixtures (does NOT author fixtures). |
| T10 | ADR for the mock-conformance guard + destructor seam | T5, T12 | low | `docs/decisions/0NN-ctps-mock-conformance-guard.md` (next free number — verify highest in `docs/decisions/`), `DECISIONS.md` | Short ADR: "the CTPS mock builder surface is kept a superset of the real surface by TWO complementary guards — a compile-time conformance TU (signature/removed drift) + a Python name-superset check (real-grew drift) — and the mock records at destructor with an `_executed` guard mirroring the real builders." State the TU's known limits (real-grew, assignability-not-exact-type). Extends ADR-030/049 (host-mock trust boundary) and ADR-033 (CTPS trace). Update DECISIONS.md index + status table. Verify no ADR-number collision before assigning. |
| T11 | End-to-end serializer→loader→renderer seam test | T1, T6, T7 | med | `WRO2026/test/CMakeLists.txt` (register), `tools/path_planner/tests/test_ctps_end_to_end.py` (new), CI job wiring | Build+run `ctps_dump --fixture` (real serializer output), pipe stdout through `trace_loader.load` + `track_builder.build`, assert the new `mode`/`heading`/`reverse`/`lineFollowDist` fields survive and a `wallStop` replays without a `float(None)` crash. CI-gated per the `test_path_tracker_parity_dump` precedent: `--ignore` in the broad pytest job, invoke by explicit path in a dedicated job that builds `ctps_dump` first. Satisfies AC13; NC15. |
| T12 | Python name-superset guard | — | low | `tools/path_planner/tests/test_ctps_mock_superset.py` (new) | libcst/regex extract public fluent-builder method names from real `WRO2026/Drive.h` + `LineFollower.h` and the `ctps_mocks/` shadows; assert each mock builder's name-set ⊇ its real counterpart (catches the real-grew #89 drift the compile TU can't). Include a negative unit test with a synthetic mock missing a method → assertion fires. Satisfies AC8 (real-grew half); NC5. |

**Parallelism:** T1, T6, T12 start immediately (T6 codes against the JSON contract in this spec, not T1's code; T12 reads headers only). After T1: T2 ∥ T3. After T2+T3: T4 ∥ T5. After T6: T7, T8. After T7+T8: T9. After T1+T6+T7: T11. After T5+T12: T10. **Contract-drift guard:** because T6 races T1, T11 is the backstop that proves the real serializer and the loader actually agree — a divergence there fails T11 even though T6 and the hand-authored T8 fixtures pass. Lanes (C++: T1→{T2,T3}→{T4,T5}; Python: T6→{T7→T9, T8}; guards: T12, T11, T10) join at CI.

## Notes for Implementer

- **The `_executed` guard is the load-bearing detail.** Copy the real builders' pattern (`Drive.h:1499-1521`, `:1752-1758`): destructor does `if (_executed) return; _executed = true;` then emits. Keep the mock non-movable (don't add a move ctor); the `static_assert` documents that, and guaranteed elision means no move is invoked on the by-value return. Result: exactly-one emit, robust even if movability is ever reintroduced.
- **Emission order:** for the canonical one-verb-per-statement discarded-temporary shape, destructor-emit lands at the statement's `;` in authored order — identical to construction order. Do not author two Drive verbs in a single full-expression (temporaries destruct in reverse construction order); no existing or authored code does this.
- **Do not model firmware default-resolution in the mock** (the epic #107 "behavioral mock" trap): the mock captures only *explicit* setter calls. A `wallStop` with no explicit `setMode` records mode-absent even though firmware defaults it to GYRO — that is the correct thin-mock behavior; the render stays a nominal forward push.
- **`_drive_label` regression risk:** existing track_builder tests assert manip labels (TC2) and drive-event presence (TC5), not exact drive labels — but keep existing method label formatting byte-identical and only *append* mode/heading conditionally, to be safe.
- **The `cm is None` guard in track_builder is mandatory** — it is the actual fix for the wallStop crash (AC7). Even after T1 emits `cm` for wallStop, keep the guard for partial/legacy traces.
- **Conformance TU wiring gotchas:** put the TU in its own file force-included/compiled only into its own target; `ctps_mocks/` is already the CTPS include root. Reuse `add_ctps_mock_force_includes` + `add_fixture_routes_force_include`. A missing setter should fail *this* target's build, surfaced by ctest.
- **`lineFollowDist` joins the raw-drive family** that advances the shared chassis without advancing the logical node, so — like `driveDist`/`stopAtLine` — it feeds the post-`goto` drift check (track_builder.py:347-357). Authored `Tasks.cpp` line-follows (`:288`, `:312`) will now contribute `drift_warn` events where before they were invisible; that is correct (the desync was always there, just unrendered). Add an NC that pins a `lineFollowDist`-then-`goto` drift event.
- **The two-phase GYRO render is a visualization choice, not physical fidelity** — real `MoveGyro` regulates heading *during* a straight (a curved arc when entry heading ≠ H), not turn-in-place-then-straight. Acceptable and clearest for near-aligned authored moves; document it in the ADR/code comment so a future reader doesn't "fix" it.
- **The mock stays non-movable while the real builders are movable** (`Drive.h:1499`) — a deliberate narrowing. CTPS only ever sees discarded temporaries, guaranteed elision means no move is invoked, and the `_executed` guard makes emission idempotent even if that changed. The `static_assert(!copy && !move)` documents the invariant; do NOT "mirror the real move ctor" — that adds machinery for a lifetime CTPS never exercises. The re-introduced end-of-statement emission order is safe only under one-verb-per-statement (already the authored convention; two Drive verbs in one full-expression would destruct in reverse order).
- **Run the FULL pytest and FULL ctest**, not per-file — the schema/contract touches both suites and the C++ self-test.
- **Optional cleanup (T6):** `trace_loader.py:109-113` doc claims unknown-drive-method commands stay inspectable via `Command.fields`, but `_parse_command` returns `None` (`:240-252`). Correct or delete that stale sentence while touching the file.
- **Rollback:** the change is additive (schema, new method, new mock members). Reverting T7/T6 restores current-facing rendering; reverting T2/T3 restores construction recording. No persisted data or field-data migration is involved.
