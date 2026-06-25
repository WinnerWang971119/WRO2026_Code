# PathTracker + Drive host-link (issue #106) Plan

**Goal:** Host-link the real `WRO2026/PathTracker.cpp` + `WRO2026/Drive.cpp` so the PathTracker
per-edge FSM and the Drive builder/queue/profile logic are asserted against production code in CI,
rewrite the five PathTracker mirror tests to drive the real singletons, and delete every
`// MIRROR: PathTracker.cpp:<line>` slice — closing the last gap in ADR-030.

**Approach:** Balanced real-link. The five rewritten targets compile the **real** `Drive.cpp` +
`PathTracker.cpp` + `Localization.cpp` + `YawSource.cpp` against the existing `hal_mocks/`
MatrixMiniR4 facade, with **thin recording-mocks** for the upper peripherals PathTracker only
*commands* — `Intersection`, `LineFollower`, `WROLineArray` — in a new `path_mocks/` dir. The
trust boundary sits at the MatrixMiniR4 vendor facade + the line-sensor leaf: the real Drive FSM
polls `MiniR4.DriveDC.isPrevTaskDone()` and the real PathTracker FSM polls the real `Drive.isIdle()`,
so each test drives a cooperative-tick loop (`Drive.update → Intersection → LineFollower →
PathTracker.update`) and **scripts the done-bit + encoders + sensor gates between ticks** (ADR-049:
no auto-evolving multi-tick mock). All five consolidate onto the one feature-complete
`fixtures/routes_fixture.cpp`, resolving nodes by name. This mirrors the `test_mission_dispatch`
recipe (#111) with **one inversion** called out below.

## Scope

- **In scope:**
  - New `WRO2026/test/path_mocks/` dir: thin recording-mocks `WROLineArray.h`, `Intersection.h`,
    `LineFollower.h` (`inline` globals, real include guards, record-and-test-set fidelity).
  - `hal_mocks/hal_mocks.cpp`: add `MockHardwareSerial Serial1;` + `uint32_t mockTickCount;`
    definitions (forced by real `Drive.cpp`'s compiled `delay()`).
  - Rewrite the five mirror tests to drive the **real** `PathTracker` + `Drive` singletons and
    **delete every `// MIRROR:` slice**: `test_path_tracker_passthrough.cpp`,
    `_bidirectional.cpp`, `_stop_at_line.cpp`, `_projection.cpp`, `_pass_through_projection.cpp`.
  - `WRO2026/test/CMakeLists.txt`: a new `add_path_force_includes(target)` helper + re-wire the
    five tests' targets to link real `Drive.cpp` + `PathTracker.cpp` + `Localization.cpp` +
    `YawSource.cpp` + `routes_fixture.cpp` + `hal_mocks.cpp` against the custom include list
    (`hal_mocks/` + `path_mocks/` first; `mocks/` omitted). Preserve the flag-paired ON/OFF target
    pairs (`PATH_PASS_THROUGH_ENABLED`, `BIDIRECTIONAL_AUTO_ENABLED`).
  - Delete the bidirectional test's `mission_replica` namespace (Mission step-queue logic now
    asserted against real `Mission.cpp` in `test_mission_dispatch`, #111).
  - Extend `routes_fixture` regenerate-script docstring's "stable-name contract" to the node names
    the five tests resolve. (No graph regeneration — the fixture already carries every needed node.)
  - Update ADR-030 status (Partially implemented → Implemented); PR body `Closes #106`.

- **Out of scope:**
  - Any change to `PathTracker.cpp` / `Drive.cpp` runtime behavior — this is a test-only change. A
    real bug the link surfaces is **reported, not silently patched** (file an issue / flag in the
    PR; present fix options before touching firmware).
  - `test_path_tracker_reset_arrival.cpp` + `test_return_to_node_dispatch.cpp` — these ALSO carry
    PathTracker FSM `// MIRROR` slices (over real `Localization` + fixture routes) but are **not**
    in this issue's named five. De-mirroring them is a natural fast-follow once real
    `PathTracker.cpp` is host-linkable (this PR makes it so) — **track as a follow-up issue**, do
    not pull into this PR.
  - `test_line_follower_overshoot` (the original real-`LineFollower.cpp` SUT) and the
    `mocks/` no-op Drive it uses — untouched. `mocks/Drive.h` stays for that target;
    `hal_mocks/Drive.h` stays for `test_mock_hal` + `test_mission_dispatch`.
  - A new ADR — this is an *application* of ADR-049 (mock HAL) + ADR-030 (link real symbols).
  - Regenerating `routes_fixture.{h,cpp}` (the fixture already covers every case) or touching
    production `routes.{h,cpp}` / `field_data`.

## Decisions

- **THE INVERSION vs #111 (load-bearing).** Because we link **real** `Drive.cpp`, the **production**
  `Drive.h` (guard `DRIVE_H`) MUST be the definition that compiles — we do **NOT** force-include
  `hal_mocks/Drive.h`, and there is no PathTracker mock. The trust boundary moves *down* one layer:
  mock the `MatrixMiniR4` facade + `WROLineArray`, and let the real `Drive.h` / `PathTracker.h` win.
  This is the **opposite** of `test_mission_dispatch` (#111), which force-included the *mock*
  `Drive.h`. Force-including `hal_mocks/Drive.h` here would replace `DriveClass` with the mock and
  real `Drive.cpp` would fail to find its own private members. (Fable consult, landmine #4.)
- **Done-bit driven via `MiniR4.DriveDC.mockSetPrevTaskDone(...)`, NOT `Drive.mockSetIdle(...)`.**
  The real `DriveClass` has no `mockSetIdle` — that knob exists only on the `hal_mocks/Drive.h`
  mock, which these targets do not link. Real `Drive::isIdle()` derives idle from its queue state +
  the lower-MCU done-bit, so a test makes Drive "busy/idle" by scripting
  `MiniR4.DriveDC.mockSetPrevTaskDone(false/true)` (+ `mockAdvanceDegrees` on `M2`/`M3` /
  `DriveDC` for distance-terminated moves). This is the single most likely reflex to get wrong
  carrying over from #111. (Fable consult, landmine #1.)
- **Balanced link depth** (user choice). Real `Drive` + `PathTracker` + `Localization` +
  `YawSource`; thin recording-mocks for `Intersection` + `LineFollower` + `WROLineArray`.
  `Localization.cpp` is already host-compiled (`test_localization_offset`, `test_reanchor_at_wall`),
  so real re-anchor (`onIntersection` / `onDeadReckonedAdvance` / `reanchorAtWall` / `clearOffset`)
  assertions come for free. Maximal (real LineFollower+Intersection) was rejected as brittle (the
  LineFollower PID couples into every nav test); minimal (mock Localization too) was rejected
  (loses the free real re-anchor coverage).
- **Per-hop `expect*` helpers, not raw per-tick loops** (Fable amendment 1). A shared test harness
  exposes semantic helpers — `cooperativeTick()`, `expectPreTurn(imuYaw, wheelByte)`,
  `expectLineHop()`, `expectBlindHop(cm)`, `expectStopAtLine(cm, found)`, with a raw `tick()`
  escape hatch for off-nominal transitions (fault mid-turn). Each helper encapsulates the multi-tick
  done-bit/sensor handshake for one FSM phase and folds the lower-MCU assert inside it. Matches the
  repo's existing `resetWorld()` helper idiom.
- **Routing guards on every consolidated test** (Fable amendment 2). Before exercising a hop, assert
  the route is forced: `ASSERT_EQ(FieldMap::getNextHop(start, goal), expectedFirstHop)` and, for a
  pass-through triple A→B→C, assert `FieldMap::getEdge(A, C).type == Routes::EdgeType::NONE` so
  Dijkstra MUST traverse B. Converts a future fixture re-weight that silently bypasses the hop under
  test into a red test. (Answers the consolidation/Dijkstra-bypass risk.)
- **Assert through to the lower-MCU `MiniR4.DriveDC` call for representative hops** (user choice /
  Fable amendment 3). A real pre-edge turn must land a `MiniR4.DriveDC.MoveGyroDegs`/`TurnGyro` with
  the resolved yaw target + wheel pattern; a seek must issue the right move. The `MockDriveDC`
  recording surface (`lastMoveGyroDegsTarget`, `lastMoveDegsDeg`, `moveGyroDegsCallCount`, …) already
  exists from #110 — no new recording fields needed. Use it for a few representative hops per test,
  not every assert, to avoid over-coupling to Drive's internal command translation.
- **Consolidate on `routes_fixture.cpp`** (user choice). All five resolve nodes by name over the one
  14-node feature-complete fixture; extend its stable-name contract. Kills the
  production/toy/inline-mock divergence (today: bidirectional→production, projection/pass-through→toy,
  passthrough→inline mock FieldMap).
- **Delete `mission_replica`** (user choice). Mission modifier no-op behavior is asserted against
  real `Mission.cpp` in `test_mission_dispatch` (#111); this test stays PathTracker-focused.
- **EventTrace stays the `hal_mocks/` no-op mock** — real `PathTracker.cpp` only calls
  `EventTrace.pushEvent(...)`, which the no-op mock satisfies; no real `EventTrace.cpp` is linked, so
  ADR-049's `MiniR4OLED.h WHITE`-macro revisit trigger never fires (satisfied-by-avoidance, same as
  #111). Do not add a `WHITE` guard.
- **No new ADR.** ADR-030 flips Partially implemented → Implemented on landing.
- **Approach consult:** Fable (run as Opus per the skill's Fable-unavailable note) **ENDORSED** the
  balanced approach with three amendments, all folded in above (per-hop helpers, routing guards,
  DriveDC done-bit/translation asserts). Codex consult **skipped** — the subagent call was rejected
  this session (run `/codex:setup` to enable). Proceeded on the Fable consult alone per the skill.

## Acceptance Criteria

- [ ] `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test
      --output-on-failure` passes on **both** local MSVC and CI g++, with all five rewritten ctests
      present and green.
- [ ] **No regression:** every previously-passing ctest still passes — especially `test_mock_hal`,
      `test_mission_dispatch`, `test_tasks_dispatch`, `ctps_dump_selftest`, `manipulator`,
      `localization_offset`, `yaw_source`, `line_follower_overshoot`, `reanchor_at_wall[_strict]`,
      `path_tracker_reset_arrival`, `return_to_node_dispatch`, and the four `*_compile_fail` targets.
- [ ] The five targets link the **real** `PathTracker.cpp` + **real** `Drive.cpp` (grep the targets'
      sources in CMakeLists). Assertions run against the production `PathTracker` / `Drive` singletons.
- [ ] **Zero `// MIRROR:` slices remain** in the five rewritten files (grep `MIRROR` returns nothing
      in them), and no `replica_*` / `mission_replica` / inline re-implementation of a PathTracker or
      Mission decision survives. Helpers that *call* the real frozen `*Policy.h` resolvers
      (`resolveDirection`, `resolveStopAtLine`, `resolveSensorOvershootTrim`, `reverseUnavailable`)
      are NOT mirrors and may stay.
- [ ] Real `Drive.h` / `PathTracker.h` win in every target — **no** target force-includes
      `hal_mocks/Drive.h` or any PathTracker mock; the `DRIVE_H` / `PATHTRACKER_H` guards resolve to
      the production headers.
- [ ] `hal_mocks.cpp` defines `Serial1` + `mockTickCount`; a `WROLineArray` mock is reachable via
      `path_mocks/` (NOT via `mocks/` on the include path — ODR mutual-exclusion holds: no target
      sees both `mocks/` and `hal_mocks/`).
- [ ] The done-bit is driven via `MiniR4.DriveDC.mockSetPrevTaskDone` (+ encoder advance) — grep the
      tests for no `Drive.mockSetIdle` (which does not exist on the real `DriveClass`).
- [ ] Each test pins its routing with a `getNextHop` guard (and the `getEdge(A,C).type == NONE`
      no-bypass guard for the pass-through triples), so a future fixture re-weight fails loudly.
- [ ] At least the representative hops assert through to the lower-MCU `MiniR4.DriveDC` call (yaw
      target / wheel pattern / distance), not only PathTracker-level outcomes.
- [ ] Per-test behavioral coverage holds (see Testing Strategy): pass-through (intersection-gated +
      distance-gated), bidirectional REVERSE_UNAVAILABLE guard + AUTO toggle, STOP_AT_LINE
      re-anchor/fault + seek-overshoot trim, projection LINE/BLIND dispatch + auto-rule toggle — all
      against the real FSM, both compile-flag builds where applicable.
- [ ] Cleanup landed: `mission_replica` deleted; `routes_fixture` stable-name contract extended;
      ADR-030 status updated; PR body `Closes #106`. Any real bug surfaced is reported, not patched.

## Data Model

Not applicable (host-test target; no runtime data structures). The shapes that matter are the mock
class surfaces and the test-harness helper API, pinned in **Contracts & Interfaces**.

## API Contracts

Not applicable (no network/API surface). The cross-file bindings are the mock class + CMake-helper +
harness-helper signatures in **Contracts & Interfaces**.

## Contracts & Interfaces

Single source of truth for every seam more than one task touches.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/test/path_mocks/WROLineArray.h` | T1 | T2, T4 (force-include) |
| `WRO2026/test/path_mocks/Intersection.h` | T1 | T2, T4 |
| `WRO2026/test/path_mocks/LineFollower.h` | T1 | T2, T4 |
| `WRO2026/test/hal_mocks/hal_mocks.cpp` (Serial1 + mockTickCount) | T2 | T4 |
| `WRO2026/test/test_path_tracker_harness.h` (shared `expect*` helpers) | T3 | T5a–T5e |
| `WRO2026/test/test_path_tracker_passthrough.cpp` | T5a | — |
| `WRO2026/test/test_path_tracker_pass_through_projection.cpp` | T5b | — |
| `WRO2026/test/test_path_tracker_projection.cpp` | T5c | — |
| `WRO2026/test/test_path_tracker_stop_at_line.cpp` | T5d | — |
| `WRO2026/test/test_path_tracker_bidirectional.cpp` | T5e | — |
| `WRO2026/test/CMakeLists.txt` | T4 | — |
| `WRO2026/test/fixtures/regenerate_routes_fixture.py` (docstring contract only) | T6 | — |
| `docs/decisions/030-harness-links-real-sources-ci-enforced.md` | T6 | — |

### The force-include recipe — `add_path_force_includes(target)` (T4 owns)

The target COMPOSES force-includes in this ORDER (mirrors the `test_mission_dispatch` ordering, with
the inversion: **NO Drive mock, NO PathTracker mock**):

```cmake
#   1. add_fixture_routes_force_include(t)   # routes_fixture.h (guard ROUTES_H) — MUST win FIRST,
#                                            #   before any header pulls production routes.h
#   2. <new> add_path_force_includes(t)      # the path_mocks/ peripherals + the hal EventTrace mock:
#        ${HAL_MOCKS_DIR}/EventTrace.h        (guard EVENTTRACE_H)  -- no-op; real PathTracker pushEvent
#        ${PATH_MOCKS_DIR}/WROLineArray.h     (guard WROLINEARRAY_H) -- real Drive.cpp + Intersection use it
#        ${PATH_MOCKS_DIR}/Intersection.h     (guard INTERSECTION_H)
#        ${PATH_MOCKS_DIR}/LineFollower.h     (guard LINEFOLLOWER_H)
#   # NO Drive.h force-include (real Drive.h wins). NO MatrixMiniR4.h force-include
#   #   (<MatrixMiniR4.h> is angle-included; the hal_mocks/-first include path picks the mock).
#   #   NO Localization.h / YawSource.h / PathTracker.h force-include (real headers win).
```

Custom include list (NOT `COMMON_INCLUDES`, which contains `mocks/`):
`${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR}` — `hal_mocks/`
and `path_mocks/` lead so the mock `<Arduino.h>`/`<MatrixMiniR4.h>` win; `mocks/` is OMITTED (ODR
mutual-exclusion). Use `/FI` (MSVC) + `"SHELL:-include …"` (g++) for every force-include.

### Target wiring (T4) — the shared link set for each of the five

```cmake
set(PATH_MOCKS_DIR ${TEST_DIR}/path_mocks)
# (repeated per target; passthrough + bidirectional are doubled for the ON/OFF compile flag)
add_executable(test_path_tracker_<name>
    ${TEST_DIR}/test_path_tracker_<name>.cpp
    ${SKETCH_DIR}/PathTracker.cpp
    ${SKETCH_DIR}/Drive.cpp
    ${SKETCH_DIR}/Localization.cpp
    ${SKETCH_DIR}/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${HAL_MOCKS_DIR}/hal_mocks.cpp)
target_include_directories(test_path_tracker_<name> PRIVATE
    ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_<name>)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_<name>)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_<name> COMMAND test_path_tracker_<name>)
```

Compile-flag dimensions (target-wide `target_compile_definitions`, exactly like today):
- `passthrough` → two targets, `PATH_PASS_THROUGH_ENABLED_DEFAULT={true,false}`.
- `bidirectional` → `BIDIRECTIONAL_AUTO_ENABLED_DEFAULT={true,false}`. **AND** `LINE_BOARD_COUNT` is
  a target-wide compile constant the real `reverseUnavailable` guard reads — the REVERSE_UNAVAILABLE
  (1-board) case and any reverse-allowed (2-board) case need their board count set per target via
  `target_compile_definitions(... LINE_BOARD_COUNT_DEFAULT=<1|2>)`. Factor the bidirectional targets
  so each board-count/flag combination a case needs has a matching binary (the implementer chooses
  the minimal target set; document which combo each TC runs under).

### Global-singleton strategy

- **`hal_mocks/hal_mocks.cpp`** (extern + defined): `Drive`? — **NO.** This target links the **real**
  `Drive` (defined in real `Drive.cpp`) and the **real** `PathTracker` (real `PathTracker.cpp`). What
  `hal_mocks.cpp` provides for these targets: `MatrixMiniR4 MiniR4`, `MockSerial Serial`,
  `uint32_t g_mockMillis`, **+ new** `MockHardwareSerial Serial1` + `uint32_t mockTickCount`. The
  `EventTraceClass EventTrace` global comes from `hal_mocks.cpp` too (its `extern` is in
  `hal_mocks/EventTrace.h`, force-included). **Do NOT** let `hal_mocks.cpp`'s `DriveClass Drive`
  definition reach these targets — it would multiply-define against real `Drive.cpp`'s `Drive`. (See
  Error Handling: `hal_mocks.cpp` defines `Drive` for the `test_mock_hal`/`test_mission_dispatch`
  world; linking it here means real `Drive.cpp`'s `Drive` and the mock `Drive` collide. **Resolution
  in T2/T4:** split the Arduino-core + `MiniR4`/`Serial1`/`mockTickCount`/`EventTrace` globals these
  targets need into a unit that does NOT also define the mock `DriveClass Drive` — e.g. these targets
  link a small `path_mocks/path_mocks.cpp` that defines only `MiniR4 / Serial / Serial1 /
  g_mockMillis / mockTickCount / EventTrace`, and do NOT link `hal_mocks.cpp`. Real `Drive.cpp`
  defines `Drive`; `path_mocks/` `inline` globals define `LineArray / Intersection / LineFollower`.)
- **`path_mocks/` facades:** `inline` globals (`LineArray`, `Intersection`, `LineFollower`) — one
  definition across the target's TUs, no extra `.cpp` for them.
- **Never link `mocks/mocks.cpp`** (defines the old-world `Drive`) into these targets.

> NOTE (resolve in T2): the cleanest factoring is a dedicated `path_mocks/path_mocks.cpp` defining the
> Arduino-core + facade globals these targets need (`MiniR4`, `Serial`, `Serial1`, `g_mockMillis`,
> `mockTickCount`, `EventTrace`) — leaving `Drive` to real `Drive.cpp`. Adding `Serial1`/`mockTickCount`
> to `hal_mocks.cpp` (the AC wording) is correct for the *symbol*, but these targets must obtain the
> facade globals from a source that does **not** also define a mock `Drive`. T2 decides: either (a) a
> new `path_mocks/path_mocks.cpp`, or (b) confirm `hal_mocks.cpp`'s `Drive` definition can be
> conditionally excluded. **(a) is recommended** — it keeps `hal_mocks.cpp` untouched for its existing
> consumers and avoids any ODR risk. If (a), the `Serial1`/`mockTickCount` definitions live in
> `path_mocks/path_mocks.cpp`, and the AC line about `hal_mocks.cpp` is satisfied by the equivalent
> `path_mocks/path_mocks.cpp` (note the deviation in the PR).

### `path_mocks/` thin-mock surfaces (T1 owns)

Each is record-and-test-set per ADR-049, with a "Fidelity — where this mock can lie" docblock; real
include guards; `inline` globals (defined in `path_mocks/path_mocks.cpp` per the note above, or
`inline` in-header — T1/T2 pick one consistently).

- **`WROLineArray.h`** — guard `WROLINEARRAY_H`, class `WROLineArrayClass`, `LineArray` global.
  Real `Drive.cpp` calls `getJunctionType()` + `setReverseActive(bool)`; real `Intersection`/
  `LineFollower` mocks reference its `JunctionType` enum. Provide: `JunctionType` enum mirroring
  production, `getJunctionType()` (test-set), `setReverseActive(bool)` (record), `getError()` /
  `isOnline()` / `begin()` / `calibrate()` as test-set or no-op. Mirror the existing
  `mocks/WROLineArray.h` surface so nothing else breaks.
- **`Intersection.h`** — guard `INTERSECTION_H`, class `IntersectionClass`, `Intersection` global.
  `isAtIntersection()` returns a test-settable flag (read-and-clear semantics: returns then resets,
  matching production), `getLastType()` test-set, `getEdgeCount()` test-set, `begin()`/`update()`
  no-op, `mockSetAtIntersection(bool)` / `mockSetLastType(JunctionType)` / `mockReset()`.
- **`LineFollower.h`** — guard `LINEFOLLOWER_H`, class `LineFollowerClass`, `LineFollower` global.
  Records the arm cascade so PathTracker's arming is assertable: `setEnabled(bool)` (record
  `lastEnabled` + count), the fluent enable builder returning `*this` with `setBrake`/`setBrakeOnGate`/
  `setDistanceTarget`/`setTShapeRampDown`/`tShape` recording their key arg, `followDistance`/
  `followUntilIntersection`. State queries test-set: `isDistanceReached()`, `isLineLost()`. No-op
  `begin()`/`update()`/`setTickCallback`. `mockSetDistanceReached(bool)` / `mockReset()` / capture
  fields (`lastEnabled`, `lastBrakeOnGate`, `lastTShapeRampDown`, `lastDistanceTarget`). Base it on
  the existing `mocks/LineFollower.h` / `task_mocks/LineFollower.h` builder shape so the `[[nodiscard]]`
  surface matches production.

### Shared test harness — `test_path_tracker_harness.h` (T3 owns)

A header-only set of helpers all five tests `#include`, sitting over the real singletons:

```cpp
// Production-order one tick. Call between scripted observation changes.
inline void cooperativeTick();   // Drive.update(); Intersection.update(); LineFollower.update();
                                 //   PathTracker.update();
inline void resetWorld();        // mockResetMiniR4(); Intersection/LineFollower/LineArray resets;
                                 //   PathTracker.begin(); Localization.begin(); mockResetMillis();

// Semantic per-hop helpers — each runs the multi-tick done-bit/sensor handshake for one FSM phase,
// asserts the lower-MCU MiniR4.DriveDC call (yaw/wheel/distance) for representative hops, and
// retires the phase (e.g. MiniR4.DriveDC.mockSetPrevTaskDone(true) to clear TURNING).
inline void expectPreTurn(int16_t imuYawTarget, uint8_t wheelByte);
inline void expectLineHop();                 // drives isAtIntersection then isDistanceReached
inline void expectBlindHop(float cm);
inline void expectStopAtLine(float cm, bool found);
inline uint8_t routedFirstHop(uint8_t start, uint8_t goal);  // FieldMap::getNextHop(start,goal)
inline void assertNoBypass(uint8_t a, uint8_t c);            // ASSERT getEdge(a,c).type == NONE
inline uint8_t nodeByName(const char* n);    // FieldMap::findNodeByName — never index
// raw escape hatch: drive a single tick + assert an off-nominal transition (fault mid-turn).
```

The done-bit driving contract: a bounded Drive move is "in flight" while
`MiniR4.DriveDC.mockSetPrevTaskDone(false)`; the helper flips it `true` (and advances encoders for
distance-terminated moves) to retire the move, then `cooperativeTick()` lets real `Drive.update()`
report idle and real `PathTracker.update()` advance. Time advances only via `mockAdvanceMillis`.

### Naming

- Dir: `WRO2026/test/path_mocks/`. Helper header: `test_path_tracker_harness.h`. CMake helper:
  `add_path_force_includes`. ctest names unchanged (`path_tracker_passthrough_on/off`,
  `path_tracker_bidirectional_on/off`, `path_tracker_stop_at_line`, `path_tracker_projection`,
  `path_tracker_pass_through_projection`) so the CI job picks them up unchanged.
- Fixture nodes by name (extend the stable-name contract): `Start`, `Inter1`, `ProjL`, `Inter2`,
  `Inter3`, `Inter4`, `Corner`, `Spur`, `ProjB`, `Zed` (plus the existing `Wall1`/`final`).

## Error Handling (build/link landmines the implementer will hit)

- **`error: 'mockSetIdle' is not a member of 'DriveClass'`** → you carried the #111 reflex. Real
  `Drive` has no `mockSetIdle`; drive idle via `MiniR4.DriveDC.mockSetPrevTaskDone(...)` + encoders.
- **Real `Drive.cpp` private-member / "DriveClass has no member" compile explosion** → you
  force-included `hal_mocks/Drive.h` (the mock `DriveClass`). REMOVE it — real `Drive.h` must win.
  Same for any PathTracker mock.
- **`multiple definition of 'Drive'` / of `EventTrace` / `MiniR4`** → a TU defining a mock `Drive`
  (`hal_mocks.cpp` or `mocks/mocks.cpp`) is linked alongside real `Drive.cpp`. Link the facade
  globals from `path_mocks/path_mocks.cpp` (which does NOT define `Drive`), not `hal_mocks.cpp`.
- **`undefined reference to Serial1 / mockTickCount`** → real `Drive.cpp` compiles a `delay()` whose
  mock body odr-uses them. Define both in the facade-globals source (`path_mocks/path_mocks.cpp`);
  do NOT paper over by calling `delay()` from the test (advance with `mockAdvanceMillis`).
- **`ROUTES_H` "too many initializers"** → fixture-routes `/FI` not first, or `mocks/` leaked onto
  the include path so production `routes.h` defined `N_NODES` first. `add_fixture_routes_force_include`
  MUST be the first force-include; verify `mocks/` is absent from the include list.
- **g++ `cannot specify -o with -c ... multiple files`** → a force-include not wrapped in CMake
  `SHELL:`. Every g++ `-include` is `"SHELL:-include <abs>"`.
- **`-Wswitch` / `/Wswitch` on a non-exhaustive enum switch** in a mock → match the production enum
  exhaustively (real `PathTracker::Fault`, `WROLineArrayClass::JunctionType`).
- **A hop test "passes" but never traversed the hop** → missing routing guard. Add
  `ASSERT_EQ(getNextHop(start,goal), expectedHop)` (+ `assertNoBypass(A,C)` for pass-throughs).
- **`Localization` seats at `START_NODE = 8` (`Start`, BLIND)** → before driving a hop from another
  node, `Localization.declarePosition(nodeByName("…"))` / re-anchor to the intended start node.
- **MSVC green but g++ red (or vice-versa)** → both are required gates. Build under local MSVC
  (`vcvars64` + `cmake -G Ninja`) AND request the CI g++ run before declaring done. A clean rebuild
  of `build/test` for the `*_compile_fail` siblings (stale partial-env build false-fails them).
- **A real `Drive.cpp` / `PathTracker.cpp` bug surfaces** → report it (issue/PR note); do NOT silently
  edit firmware to make a test pass. Present fix options first.

## Testing Strategy

**Levels:** Host C++ behavioral integration (ctest), driving the real `PathTracker` + `Drive` +
`Localization` singletons through the cooperative-tick harness. Framework: `TestAssert.h` `ASSERT_*`
(exit 0 = pass). Each case `resetWorld()`s, re-anchors `Localization` to the intended start node,
pins routing with a `getNextHop` guard, then ticks the FSM, scripting the done-bit + sensor gates.
No hardware, no Python. Helpers that call the real frozen `*Policy.h` resolvers are kept (not mirrors).

Concrete fixture hops (by name) each test drives:

| Test (file) | Fixture hop(s) | Forced because |
|---|---|---|
| `passthrough` (PATH_PASS_THROUGH on/off) | `Inter2→Inter3→Inter4` line-line-line; `goTo(Inter4)` from `Inter2` | `getEdge(Inter2,Inter4).type==NONE` |
| `pass_through_projection` | `Inter1→ProjL→Inter2` line-proj-line; `goTo(Inter2)` from `Inter1`; also `goTo(ProjL)` (not-final → stop) | `getEdge(Inter1,Inter2).type==NONE` |
| `projection` | LINE→proj `Inter1→ProjL` (distance follow + dead-reckon); BLIND→proj `Corner→ProjB` (auto-seek + re-anchor); BLIND→non-proj negative | local dest-node read |
| `stop_at_line` | BLIND seek `Start→Inter1` (`stop_at_line="on"`): success re-anchor + DRIVE_FAULT on no-line; seek-overshoot trim on the sensor-regime pivot `Corner→ProjB→Zed` | edge override + pivot table |
| `bidirectional` (BIDIRECTIONAL_AUTO on/off; LINE_BOARD_COUNT per target) | rear-override LINE `Inter1→Spur` (`direction="rear"`) → REVERSE_UNAVAILABLE on 1-board, allowed on 2-board; AUTO toggle heading branch | edge `direction` override + board-count compile-def |

| ID | Test Case | File | Expected Behavior |
|----|-----------|------|-------------------|
| TC-PT1 | line-line-line pass-through FIRES (flag ON) | passthrough_on | `goTo(Inter4)` from `Inter2`: real FSM line-follows past `Inter3` WITHOUT brake; `Localization.onDeadReckonedAdvance`/`onIntersection` advances `Inter2→Inter3→Inter4`; assert LineFollower armed with `brakeOnGate==false` at the mid junction. |
| TC-PT2 | pass-through SUPPRESSED (flag OFF) | passthrough_off | Same fixture: at `Inter3` the FSM brakes + re-plans (stop-and-go); LineFollower armed with `brakeOnGate==true`. Proves the compile flag changes runtime behavior. |
| TC-PP1 | line-proj-line distance-gated pass-through FIRES | pass_through_projection | `goTo(Inter2)` from `Inter1`: PROJECTION_FOLLOW gate trips on distance at `ProjL`, stamps `onDeadReckonedAdvance(ProjL)` mid-follow, continues to `Inter2` re-anchor. |
| TC-PP2 | `goTo(ProjL)` not-final → STOP | pass_through_projection | ProjL IS the goal → not-final guard rejects pass-through → stops at `ProjL`. |
| TC-PP3 | BLIND-flanked / non-collinear projection → no pass-through | pass_through_projection | Outbound flank not LINE, or non-collinear → gate does not fire → stop at `ProjL`. |
| TC-PJ1 | LINE→projection dispatch | projection | `Inter1→ProjL`: real `_dispatchEdgeAfterTurn` → PROJECTION_FOLLOW distance gate → dead-reckon advance (no junction). |
| TC-PJ2 | BLIND→projection auto-seek + re-anchor | projection | `Corner→ProjB` (`BLIND_PROJECTION_AUTO_SEEK` + `isProjectionNode`): real FSM issues `Drive.stopAtLine` (assert the lower-MCU seek move); on `isLineFound` → full-advance re-anchor via `onIntersection`. |
| TC-PJ3 | BLIND→non-projection does NOT auto-seek | projection | A BLIND hop into a non-projection node → BLIND_FORWARD dead-reckon, no seek. |
| TC-PJ4 | auto-rule toggle gates the dest read | projection | Compose: with `BLIND_PROJECTION_AUTO_SEEK` effectively off the same hop dead-reckons. |
| TC-SL1 | STOP_AT_LINE success re-anchor | stop_at_line | `Start→Inter1` (`stop_at_line="on"`): real FSM seeks; script `isLineFound=true` + done-bit → `Localization.onIntersection(Inter1)` re-anchor; FSM advances. |
| TC-SL2 | STOP_AT_LINE → DRIVE_FAULT on no-line | stop_at_line | Same seek; script `Drive` `LINE_NOT_FOUND` fault (via `MiniR4.DriveDC` no-line + the real seek's fault path) → real PathTracker `Fault::DRIVE_FAULT`, FSM halts. |
| TC-SL3 | seek-overshoot trim (sensor-regime pivot) | stop_at_line | Seek into the `Corner→ProjB→Zed` sensor pivot: assert the lower-MCU seek distance reflects `resolveSensorOvershootTrim` (real frozen resolver kept). |
| TC-SL4 | resolver truth-table (KEPT, not mirror) | stop_at_line | The existing `resolveStopAtLine` / `resolveSensorOvershootTrim` direct-call TCs stay green (they call the real frozen resolvers). |
| TC-BD1 | REVERSE_UNAVAILABLE on rear LINE, 1-board | bidirectional (1-board) | `goTo(Spur)` from `Inter1` (rear-forced LINE) on `LINE_BOARD_COUNT=1` → real `_dispatchEdgeAfterTurn` raises `Fault::REVERSE_UNAVAILABLE` (from/dest set). |
| TC-BD2 | rear LINE allowed on 2-board | bidirectional (2-board) | Same hop on `LINE_BOARD_COUNT=2` → no fault; rear dispatch proceeds. |
| TC-BD3 | AUTO toggle changes direction resolution | bidirectional on/off | The `BIDIRECTIONAL_AUTO_ENABLED` flag flips the resolved direction on the same fixture heading; assert via the real `resolveDirection` path the FSM took. |
| TC-REG | no regression | (all prior) | Every previously-green ctest still passes. |

**Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest
--test-dir build/test --output-on-failure` (locally inside `vcvars64` + `-G Ninja`; CI runs g++ on
ubuntu).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | `path_mocks/` thin peripheral mocks | — | med | `WRO2026/test/path_mocks/WROLineArray.h`, `.../Intersection.h`, `.../LineFollower.h` | Author the three record-and-test-set mocks per Contracts → `path_mocks/` surfaces: real guards (`WROLINEARRAY_H`/`INTERSECTION_H`/`LINEFOLLOWER_H`), production-matching enums (`JunctionType`) + `[[nodiscard]]` builder shapes (base on existing `mocks/`/`task_mocks/` versions), test-set state (`isAtIntersection` read-and-clear, `isDistanceReached`, `getJunctionType`) + capture (`setEnabled`/`setBrakeOnGate`/`setReverseActive`/`lastEnabled`), `mockReset`. Doxygen + "Fidelity" docblock each. Globals declared `extern` here (defined in T2's `path_mocks.cpp`) OR `inline` — pick one, consistent with T2. Satisfies the `path_mocks/` ACs. |
| T2 | Facade-globals source + `Serial1`/`mockTickCount` | T1 | med | `WRO2026/test/path_mocks/path_mocks.cpp` (recommended) **or** `hal_mocks/hal_mocks.cpp` | Provide ONE TU these targets link that defines the facade/Arduino globals WITHOUT a mock `DriveClass Drive`: `MatrixMiniR4 MiniR4`, `MockSerial Serial`, `uint32_t g_mockMillis`, `MockHardwareSerial Serial1`, `uint32_t mockTickCount`, `EventTraceClass EventTrace`, and the `path_mocks/` `LineArray`/`Intersection`/`LineFollower` globals (if extern). Decide (a) new `path_mocks/path_mocks.cpp` [recommended — leaves `hal_mocks.cpp` untouched] vs (b) conditional exclusion in `hal_mocks.cpp`; document the choice. Real `Drive` comes from real `Drive.cpp`. Satisfies the Serial1/mockTickCount + no-multiple-Drive ACs. |
| T3 | Shared harness `test_path_tracker_harness.h` | T1, T2 | high | `WRO2026/test/test_path_tracker_harness.h` | Author the header-only helpers per Contracts → harness: `cooperativeTick()` (production tick order), `resetWorld()`, `expectPreTurn`/`expectLineHop`/`expectBlindHop`/`expectStopAtLine` (each runs the done-bit/sensor handshake + asserts the representative lower-MCU `MiniR4.DriveDC` call + retires the phase), `routedFirstHop`/`assertNoBypass`/`nodeByName`, raw `tick()` escape hatch. Done-bit driven via `MiniR4.DriveDC.mockSetPrevTaskDone` + encoder advance; time via `mockAdvanceMillis`. This is the load-bearing readability layer. Satisfies the harness/done-bit/lower-MCU-assert ACs. |
| T4 | CMake: `add_path_force_includes` + re-wire 5 targets | T1, T2, T3 | high | `WRO2026/test/CMakeLists.txt` | Add `PATH_MOCKS_DIR`; author `add_path_force_includes(target)` force-including (in order) `hal_mocks/EventTrace.h` + the three `path_mocks/` peripherals (`/FI` MSVC, `SHELL:-include` g++) — **NO Drive.h, NO PathTracker.h, NO MatrixMiniR4.h** force-include. Re-wire the five targets (passthrough on/off, bidirectional on/off + board-count dimension, stop_at_line, projection, pass_through_projection) to link real `Drive.cpp`+`PathTracker.cpp`+`Localization.cpp`+`YawSource.cpp`+`routes_fixture.cpp`+the T2 globals TU; custom include list (`hal_mocks/`+`path_mocks/` first, `mocks/` OMITTED); `add_fixture_routes_force_include` FIRST then `add_path_force_includes`; preserve ctest names + the ON/OFF flag defs; add `LINE_BOARD_COUNT_DEFAULT` per board-count target. Satisfies the link/ODR/inversion ACs. |
| T5a | Rewrite `test_path_tracker_passthrough.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_passthrough.cpp` | Delete `wrapDelta`/`computePassThrough`/`armLineFollowerForOvershoot` MIRROR slices. Drive real PathTracker over `Inter2→Inter3→Inter4` with routing guard + `assertNoBypass(Inter2,Inter4)`; TC-PT1 (flag ON fires) / TC-PT2 (flag OFF brakes) asserting LineFollower arm (`brakeOnGate`) + advance. Real both ON/OFF builds. |
| T5b | Rewrite `test_path_tracker_pass_through_projection.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_pass_through_projection.cpp` | Delete `bodyHeadingFor`/`replica_evaluateProjectionPassThrough`/`resolveInboundLineHop` MIRRORs. Drive real PathTracker over `Inter1→ProjL→Inter2` + guards; TC-PP1/PP2/PP3. |
| T5c | Rewrite `test_path_tracker_projection.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_projection.cpp` | Delete `replica_dispatchProjectionCell` + `firmwareAutoRuleActive`/`resolveBareBlindHop` MIRRORs (keep the real-accessor calls). Drive real PathTracker: TC-PJ1 (`Inter1→ProjL`), TC-PJ2 (`Corner→ProjB` auto-seek, assert lower-MCU seek + re-anchor), TC-PJ3, TC-PJ4. |
| T5d | Rewrite `test_path_tracker_stop_at_line.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_stop_at_line.cpp` | Delete `replica_stopAtLineTick`/`replica_dispatchDrainOrdering`/`replica_seekOvershoot` FSM-slice MIRRORs. **KEEP** the real-resolver truth-table TCs (TC-SL4). Drive real PathTracker `Start→Inter1` seek: TC-SL1 (success re-anchor), TC-SL2 (DRIVE_FAULT on no-line), TC-SL3 (seek-overshoot trim on `Corner→ProjB→Zed`). |
| T5e | Rewrite `test_path_tracker_bidirectional.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_bidirectional.cpp` | **Delete the `mission_replica` namespace** (covered by #111) and the `replica_dispatchEdgeAfterTurnGuard` MIRROR. Drive real PathTracker over the rear-override `Inter1→Spur`: TC-BD1 (REVERSE_UNAVAILABLE 1-board), TC-BD2 (allowed 2-board), TC-BD3 (AUTO toggle). Coordinate the board-count/flag target matrix with T4. |
| T6 | Cleanup: stable-name contract + ADR-030 | T5a–T5e | low | `WRO2026/test/fixtures/regenerate_routes_fixture.py` (docstring), `docs/decisions/030-harness-links-real-sources-ci-enforced.md` | Extend the fixture regenerate-script's "Decoupling contract" docstring to list the node names the five tests now resolve (so a future fixture edit knows not to rename them). Flip ADR-030 status Partially implemented → Implemented, refresh its "remaining gap" / `See also` lines (PathTracker/Drive now linked). PR body `Closes #106` + note the Codex-consult-skipped + any deviation (T2 choice) + any reported real bug. |
| T7 | Build + run both toolchains; no-regression sweep | T4, T5a–T5e, T6 | med | — | `cmake -S WRO2026/test -B build/test -G Ninja` inside `vcvars64`; build + `ctest -C Debug --output-on-failure`; confirm all five rewritten ctests + ALL prior targets green (incl. the four `*_compile_fail` on a clean rebuild). Document g++ concerns (`SHELL:-include`, `-Wswitch`) for the CI gate; request the CI g++ run. If a real Drive/PathTracker bug surfaces, STOP and report (do not patch). |

T1 and (the global-defs half of) T2 are near-independent; T2 depends on T1 only for the `path_mocks/`
global declarations. T3 needs T1+T2 (the mock surfaces + globals to compile against). T4 needs
T1+T2+T3 (files must exist). T5a–T5e are independent of each other (distinct files) and all need
T3+T4 (harness + buildable targets). T6 edits distinct files (fixture docstring + ADR) — no conflict
with T5*. T7 is the verification gate after everything. No two tasks edit the same file.

## Notes for Implementer

- **The inversion is the whole trick.** Real `Drive.h` + `PathTracker.h` WIN. Do NOT force-include
  `hal_mocks/Drive.h` or any PathTracker mock. Mock only the layer *below* Drive (MatrixMiniR4 facade
  + WROLineArray) and the peripherals PathTracker commands (Intersection, LineFollower). This is the
  opposite of `test_mission_dispatch` (#111) — re-reading that target's wiring will mislead you on
  exactly this point.
- **Drive "idle/busy" = `MiniR4.DriveDC.mockSetPrevTaskDone`, never `Drive.mockSetIdle`.** The real
  `DriveClass` has no mock knobs. Read `Drive::isIdle()` in `Drive.cpp` first to learn whether it
  gates on `_queueCount` + the done-bit or just the done-bit, and script accordingly.
- **The whole point is real-link fidelity.** Assert against the production `PathTracker`/`Drive`
  singletons; never re-introduce a slice. If a test is awkward, improve the harness/mock
  observability — do not re-mirror. Helpers that *call* the real `*Policy.h` resolvers are fine.
- **Routing is Dijkstra, not free choice.** `goTo(goal)` walks the fixture's frozen `NEXT_HOP`; to
  exercise a hop, pick `(start, goal)` whose shortest path contains it, and pin it with the
  `getNextHop` guard. The pass-through triples are forced because their bypass edge is `NONE` —
  assert that with `assertNoBypass`.
- **Re-anchor `Localization` to the intended start node** (`START_NODE = 8`/`Start` after `begin()`)
  before driving a hop from elsewhere.
- **Two toolchains, always.** Local MSVC (`cl 19.44` via `vcvars64` + `-G Ninja`) is the fast loop;
  CI g++ is the second required gate. Build under MSVC before declaring done; request the CI g++ run.
  Watch `-Wswitch` on any mock enum switch; clean-rebuild for the `*_compile_fail` siblings.
- **Report-not-patch.** This is the first time real `Drive.cpp` + `PathTracker.cpp` run in CI; a
  latent bug is plausible. If one surfaces, file it / flag it in the PR and present fix options — do
  not silently edit firmware to turn a test green.
- **Rollback:** the change is additive (one new dir + one globals TU + one harness header + CMake
  helper + test rewrites + doc refresh). If a rewritten target can't go green on both toolchains, it
  can be temporarily excluded from the default build behind a fast-follow without touching the other
  targets; the deleted MIRROR slices live in git history.
