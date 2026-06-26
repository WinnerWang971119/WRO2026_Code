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
  - New `WRO2026/test/path_mocks/path_mocks.cpp`: the single facade-globals TU these five targets
    link — defines `MiniR4` / `Serial` / `Serial1` / `g_mockMillis` / `mockTickCount` / `EventTrace`,
    and crucially does NOT define `DriveClass Drive` (real `Drive.cpp` is the sole definer). `Serial1` +
    `mockTickCount` are added HERE (forced by real `Drive.cpp`'s compiled `delay()`), NOT in
    `hal_mocks.cpp` — which stays untouched for its `test_mock_hal` / `test_mission_dispatch` consumers.
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
  - Update ADR-030 to **Implemented** but **scope the claim**: its status line + How-to-apply record
    that the five named PathTracker mirror tests now link real `PathTracker.cpp`/`Drive.cpp` (closing
    #106), while `test_path_tracker_reset_arrival.cpp` + `test_return_to_node_dispatch.cpp` retain
    narrowly-scoped PathTracker FSM `// MIRROR:` slices tracked by a named follow-up issue (do NOT
    leave the ADR asserting zero mirrored FSM logic remains). PR body `Closes #106`.

- **Out of scope:**
  - Any change to ANY production file — this is a **strictly test-only** change (no `ConfigLine.h` /
    `Config` edit; the REVERSE_UNAVAILABLE 1-board path is asserted at the frozen-resolver level
    instead — see Testing Strategy / TC-BD1). A real bug the link surfaces is **reported, not silently
    patched** (file an issue / flag in the PR; present fix options before touching firmware).
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
- **REVERSE_UNAVAILABLE (1-board) asserted at the resolver level, NO production edit** (user choice,
  Phase 5.8). The real `reverseUnavailable` guard reads `Config::LINE_BOARD_COUNT` (a compile-time
  `constexpr` = 2 with no per-target override hook), so a 1-board REVERSE_UNAVAILABLE fault cannot be
  driven through the real FSM without editing `ConfigLine.h`. To keep the change strictly test-only,
  the 1-board fault is asserted via a direct call to the frozen `reverseUnavailable(REAR, LINE, 1)`
  Policy (a KEPT resolver test — the same symbol `PathTracker.cpp`'s guard calls); the real FSM drives
  only the 2-board-allowed + AUTO-toggle cases. Adding the `_DEFAULT` hook was considered and declined
  (no production edit).
- **No new ADR.** ADR-030 flips to Implemented on landing **with an explicit scoped note** that two
  PathTracker host tests (`reset_arrival`, `return_to_node_dispatch`) still carry FSM `// MIRROR:`
  slices, tracked as a follow-up issue — the ADR must not claim zero mirrored FSM logic remains.
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
- [ ] `path_mocks/path_mocks.cpp` (the facade-globals TU these targets link, NOT `hal_mocks.cpp`)
      defines `Serial1` + `mockTickCount` (+ `MiniR4` / `Serial` / `g_mockMillis` / `EventTrace`) and
      does NOT define `Drive`; a `WROLineArray` mock is reachable via `path_mocks/` (NOT via `mocks/`
      on the include path — ODR mutual-exclusion holds: no target sees both `mocks/` and `hal_mocks/`).
- [ ] No production file is edited (the diff is confined to `WRO2026/test/` + the ADR + the fixture
      docstring). The REVERSE_UNAVAILABLE 1-board path is asserted via a direct
      `reverseUnavailable(REAR, LINE, 1)` resolver call (TC-BD1), not a 1-board real-FSM build.
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
- [ ] Cleanup landed: `mission_replica` deleted; `routes_fixture` stable-name contract extended to all
      ten names the tests resolve (incl. `Corner` + `Zed`); ADR-030 flipped to Implemented WITH the
      scoped note (two FSM mirror tests deferred to a follow-up); PR body `Closes #106`. Any real bug
      surfaced is reported, not patched.

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
| `WRO2026/test/path_mocks/path_mocks.cpp` (facade globals incl. Serial1 + mockTickCount; NOT Drive) | T2 | T4 (link) |
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
- `bidirectional` → two targets, `BIDIRECTIONAL_AUTO_ENABLED_DEFAULT={true,false}` — both on the
  default 2-board config. No board-count dimension: `LINE_BOARD_COUNT` has no per-target override hook
  and we do not add one (strictly test-only), so the REVERSE_UNAVAILABLE 1-board fault is asserted at
  the frozen-`reverseUnavailable`-resolver level (TC-BD1), not through a 1-board real-FSM build.

### Global-singleton strategy

**Resolved: these five targets link a dedicated `path_mocks/path_mocks.cpp` for facade globals, and do
NOT link `hal_mocks.cpp`.** Rationale: `hal_mocks.cpp` defines a MOCK `DriveClass Drive` (hal_mocks.cpp
line 50); linking it alongside real `Drive.cpp` (which defines the real `Drive`, Drive.cpp line ~34)
multiply-defines the symbol. So:

- **`path_mocks/path_mocks.cpp`** defines exactly: `MatrixMiniR4 MiniR4`, `MockSerial Serial`,
  `uint32_t g_mockMillis`, `MockHardwareSerial Serial1`, `uint32_t mockTickCount`, and
  `EventTraceClass EventTrace` (its `extern` lives in the force-included `hal_mocks/EventTrace.h`). It
  does **NOT** define `Drive`. It MUST resolve its headers under the SAME include order as the test
  TUs (`hal_mocks/`-first, with `add_path_force_includes` applied to it) so its
  `MiniR4`/`EventTrace`/`Serial1` types match the test TUs — compile this TU with the target's
  force-include set, or have it `#include "MatrixMiniR4.h"` + `"EventTrace.h"` + `"Arduino.h"` against
  the `hal_mocks/`-first path.
- **Real `Drive.cpp`** is the sole definer of `Drive`; **real `PathTracker.cpp`** of `PathTracker`;
  **real `Localization.cpp`** of `Localization`; **real `YawSource.cpp`** of `YawSource`.
- **`path_mocks/` peripheral facades** declare their globals (`LineArray`, `Intersection`,
  `LineFollower`) `inline` in-header (the `mocks/` / `task_mocks/` / `mmL` idiom) — one definition
  across the target's TUs, nothing extra in `path_mocks.cpp`, and it breaks any T1→T2 ordering coupling.
- **Never link `hal_mocks.cpp` or `mocks/mocks.cpp`** (both define a mock `Drive`) into these targets.

(Rejected alternative, kept on record only: conditionally excluding `hal_mocks.cpp`'s `Drive`
definition — it churns `hal_mocks.cpp` and risks its existing `test_mock_hal`/`test_mission_dispatch`
consumers, so `path_mocks/path_mocks.cpp` is chosen instead.)

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
  `followUntilIntersection`. State queries test-set: `isDistanceReached()`, `isLineLost()`. **Also
  provide `isEnabled()` (test-set) and the `setEnabled(bool).setBrake(bool)` chain** — real `Drive.cpp`'s
  `_dispatchNext` race-guard calls `LineFollower.isEnabled()` + `setEnabled(false).setBrake(false)`
  (Drive.cpp ~1607-1609), so omitting them fails the real `Drive.cpp` link. No-op
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

**Yaw-frame caveat:** the recorded `MiniR4.DriveDC.lastMoveGyroDegsTarget` is the HARDWARE-frame yaw —
real `YawSource.toHwHeading` applies `Config::IMU_YAW_SIGN` (= -1), so it equals
`IMU_YAW_SIGN * appHeading`. Scripted IMU yaw is set via `MiniR4.mmL.mockYawDeg` (also raw/hw frame);
a lower-MCU assert written in the app frame is off by the sign. `expectPreTurn` takes the app-frame
target and converts via `Config::IMU_YAW_SIGN` so the test reads in the app frame.

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
| `bidirectional` (BIDIRECTIONAL_AUTO on/off; 2-board) | rear-override LINE `Inter1→Spur` (`direction="rear"`) allowed through the real FSM on 2-board; AUTO toggle heading branch; REVERSE_UNAVAILABLE (1-board) asserted at the resolver level | edge `direction` override + frozen `reverseUnavailable` resolver |

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
| TC-PJ4 | auto-rule is a local dest-node read | projection | `BLIND_PROJECTION_AUTO_SEEK` is graph DATA baked into `routes_fixture.h` (NOT a Config `_DEFAULT` compile hook), so it cannot be toggled per-target without a second fixture. Assert the auto-rule at the resolver level — a direct call proving `isProjectionNode(dest) && BLIND_PROJECTION_AUTO_SEEK` drives the seek decision (a KEPT resolver test, not a real-FSM toggle). Do NOT fabricate a per-target toggle. |
| TC-SL1 | STOP_AT_LINE success re-anchor | stop_at_line | `Start→Inter1` (`stop_at_line="on"`): real FSM seeks; script `isLineFound=true` + done-bit → `Localization.onIntersection(Inter1)` re-anchor; FSM advances. |
| TC-SL2 | STOP_AT_LINE → DRIVE_FAULT on no-line | stop_at_line | Drive the real seek to its no-line fault: keep `LineArray.getJunctionType()` returning NONE (no crossing) and advance the encoder PAST `expected + STOP_AT_LINE_MAX_MARGIN_CM` over enough ticks so real `Drive`'s seek latches `Fault::LINE_NOT_FOUND` → real PathTracker `STOP_AT_LINE` state reads `Drive.hasFault()` → `Fault::DRIVE_FAULT`, FSM halts. (`expectStopAtLine(cm, found=false)` encapsulates the over-advance + tick budget.) |
| TC-SL3 | seek-overshoot trim (sensor-regime pivot) | stop_at_line | Seek into the `Corner→ProjB→Zed` sensor pivot: assert the lower-MCU seek distance reflects `resolveSensorOvershootTrim` (real frozen resolver kept). |
| TC-SL4 | resolver truth-table (KEPT, not mirror) | stop_at_line | The existing `resolveStopAtLine` / `resolveSensorOvershootTrim` direct-call TCs stay green (they call the real frozen resolvers). |
| TC-BD1 | REVERSE_UNAVAILABLE guard (resolver-level, KEPT) | bidirectional | Direct call to the frozen `reverseUnavailable(Direction::REAR, EdgeType::LINE, /*boards=*/1)` → true (and `(REAR, LINE, 2)` / `(REAR, BLIND, 1)` / `(FRONT, …)` → false). This is the SAME symbol `PathTracker.cpp`'s pre-turn guard calls (PathTracker.cpp ~1569-1573); no 1-board build, no production edit. NOT a mirror — it exercises the real Policy directly. |
| TC-BD2 | rear LINE allowed on 2-board | bidirectional (2-board) | Same hop on `LINE_BOARD_COUNT=2` → no fault; rear dispatch proceeds. |
| TC-BD3 | AUTO toggle changes direction resolution | bidirectional on/off | The `BIDIRECTIONAL_AUTO_ENABLED` flag flips the resolved direction on the same fixture heading; assert via the real `resolveDirection` path the FSM took. |
| TC-REG | no regression | (all prior) | Every previously-green ctest still passes. |

**Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest
--test-dir build/test --output-on-failure` (locally inside `vcvars64` + `-G Ninja`; CI runs g++ on
ubuntu).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | `path_mocks/` thin peripheral mocks | — | med | `WRO2026/test/path_mocks/WROLineArray.h`, `.../Intersection.h`, `.../LineFollower.h` | Author the three record-and-test-set mocks per Contracts → `path_mocks/` surfaces: real guards (`WROLINEARRAY_H`/`INTERSECTION_H`/`LINEFOLLOWER_H`), production-matching enums (`JunctionType`) + `[[nodiscard]]` builder shapes (base on existing `mocks/`/`task_mocks/` versions), test-set state (`isAtIntersection` read-and-clear, `isDistanceReached`, `getJunctionType`) + capture (`setEnabled`/`setBrakeOnGate`/`setReverseActive`/`lastEnabled`), `mockReset`. Doxygen + "Fidelity" docblock each. Declare the `LineArray`/`Intersection`/`LineFollower` globals `inline` in-header (the `mocks/`/`task_mocks/`/`mmL` idiom) so no definition is needed in `path_mocks.cpp` and T2 stays decoupled. Satisfies the `path_mocks/` ACs. |
| T2 | Facade-globals TU `path_mocks/path_mocks.cpp` | — | med | `WRO2026/test/path_mocks/path_mocks.cpp` | Author the ONE facade-globals TU these targets link, defining the globals WITHOUT a mock `DriveClass Drive`: `MatrixMiniR4 MiniR4`, `MockSerial Serial`, `uint32_t g_mockMillis`, `MockHardwareSerial Serial1`, `uint32_t mockTickCount`, `EventTraceClass EventTrace`. The `path_mocks/` `LineArray`/`Intersection`/`LineFollower` globals are `inline` in their headers (T1), so they need NO definition here — which decouples T2 from T1. Real `Drive` comes from real `Drive.cpp`. Compile this TU under the same `hal_mocks/`-first include order / force-include set as the test TUs so its types match. (`hal_mocks.cpp` is NOT linked into these targets; the rejected "conditionally exclude `hal_mocks.cpp`'s Drive" alternative is recorded in Contracts.) Satisfies the Serial1/mockTickCount + no-multiple-Drive ACs. |
| T3 | Shared harness `test_path_tracker_harness.h` | T1, T2 | high | `WRO2026/test/test_path_tracker_harness.h` | Author the header-only helpers per Contracts → harness: `cooperativeTick()` (production tick order), `resetWorld()`, `expectPreTurn`/`expectLineHop`/`expectBlindHop`/`expectStopAtLine` (each runs the done-bit/sensor handshake + asserts the representative lower-MCU `MiniR4.DriveDC` call + retires the phase), `routedFirstHop`/`assertNoBypass`/`nodeByName`, raw `tick()` escape hatch. Done-bit driven via `MiniR4.DriveDC.mockSetPrevTaskDone` + encoder advance; time via `mockAdvanceMillis`. This is the load-bearing readability layer. Satisfies the harness/done-bit/lower-MCU-assert ACs. |
| T4 | CMake: `add_path_force_includes` + re-wire 5 targets | T1, T2, T3 | high | `WRO2026/test/CMakeLists.txt` | Add `PATH_MOCKS_DIR`; author `add_path_force_includes(target)` force-including (in order) `hal_mocks/EventTrace.h` + the three `path_mocks/` peripherals (`/FI` MSVC, `SHELL:-include` g++) — **NO Drive.h, NO PathTracker.h, NO MatrixMiniR4.h** force-include. Re-wire the five tests' targets (passthrough on/off, bidirectional on/off, stop_at_line, projection, pass_through_projection — all on the default 2-board config, no board-count dimension) to link real `Drive.cpp`+`PathTracker.cpp`+`Localization.cpp`+`YawSource.cpp`+`routes_fixture.cpp`+the T2 globals TU; custom include list (`hal_mocks/`+`path_mocks/` first, `mocks/` OMITTED); `add_fixture_routes_force_include` FIRST then `add_path_force_includes`; preserve ctest names + the ON/OFF flag defs. Satisfies the link/ODR/inversion ACs. |
| T5a | Rewrite `test_path_tracker_passthrough.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_passthrough.cpp` | Delete `wrapDelta`/`computePassThrough`/`armLineFollowerForOvershoot` MIRROR slices. Drive real PathTracker over `Inter2→Inter3→Inter4` with routing guard + `assertNoBypass(Inter2,Inter4)`; TC-PT1 (flag ON fires) / TC-PT2 (flag OFF brakes) asserting LineFollower arm (`brakeOnGate`) + advance. Real both ON/OFF builds. |
| T5b | Rewrite `test_path_tracker_pass_through_projection.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_pass_through_projection.cpp` | Delete `bodyHeadingFor`/`replica_evaluateProjectionPassThrough`/`resolveInboundLineHop` MIRRORs. Drive real PathTracker over `Inter1→ProjL→Inter2` + guards; TC-PP1/PP2/PP3. |
| T5c | Rewrite `test_path_tracker_projection.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_projection.cpp` | Delete `replica_dispatchProjectionCell` + `firmwareAutoRuleActive`/`resolveBareBlindHop` MIRRORs (keep the real-accessor calls). Drive real PathTracker: TC-PJ1 (`Inter1→ProjL`), TC-PJ2 (`Corner→ProjB` auto-seek, assert lower-MCU seek + re-anchor), TC-PJ3, TC-PJ4. |
| T5d | Rewrite `test_path_tracker_stop_at_line.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_stop_at_line.cpp` | Delete `replica_stopAtLineTick`/`replica_dispatchDrainOrdering`/`replica_seekOvershoot` FSM-slice MIRRORs. **KEEP** the real-resolver truth-table TCs (TC-SL4). Drive real PathTracker `Start→Inter1` seek: TC-SL1 (success re-anchor), TC-SL2 (DRIVE_FAULT on no-line), TC-SL3 (seek-overshoot trim on `Corner→ProjB→Zed`). |
| T5e | Rewrite `test_path_tracker_bidirectional.cpp` | T3, T4 | high | `WRO2026/test/test_path_tracker_bidirectional.cpp` | **Delete the `mission_replica` namespace** (covered by #111). KEEP `replica_dispatchEdgeAfterTurnGuard`'s call to the frozen `reverseUnavailable` resolver as a direct resolver test (TC-BD1 — NOT a mirror; it exercises the real Policy). Drive real PathTracker over the rear-override `Inter1→Spur` on 2-board: TC-BD2 (rear allowed), TC-BD3 (AUTO toggle on/off heading branch). No board-count target split, no production edit. |
| T6 | Cleanup: stable-name contract + ADR-030 scoped note | T5a–T5e | low | `WRO2026/test/fixtures/regenerate_routes_fixture.py` (docstring), `docs/decisions/030-harness-links-real-sources-ci-enforced.md` | Extend the fixture regenerate-script's "Decoupling contract" docstring to list ALL ten node names the five tests now resolve (`Start`, `Inter1`, `ProjL`, `Inter2`, `Inter3`, `Inter4`, `Corner`, `Spur`, `ProjB`, `Zed`) so a future fixture edit knows not to rename them. Flip ADR-030 to **Implemented WITH a scoped note**: edit its status line + Rule/How-to-apply to record the five named tests now link real `PathTracker.cpp`/`Drive.cpp` (closing #106) AND that `test_path_tracker_reset_arrival` + `test_return_to_node_dispatch` still carry FSM `// MIRROR:` slices tracked by a follow-up — the ADR must NOT claim zero mirrored FSM logic remains. PR body `Closes #106` + note Codex-consult-skipped + the T2 `path_mocks.cpp` choice + any reported real bug. |
| T7 | Build + run both toolchains; no-regression sweep | T4, T5a–T5e, T6 | med | — | `cmake -S WRO2026/test -B build/test -G Ninja` inside `vcvars64`; build + `ctest -C Debug --output-on-failure`; confirm all five rewritten ctests + ALL prior targets green (incl. the four `*_compile_fail` on a clean rebuild). Document g++ concerns (`SHELL:-include`, `-Wswitch`) for the CI gate; request the CI g++ run. If a real Drive/PathTracker bug surfaces, STOP and report (do not patch). |

T1 (`path_mocks/` mocks with `inline` globals) and T2 (`path_mocks/path_mocks.cpp` facade globals) are
independent and run in parallel — pinning the peripheral globals `inline` in T1's headers means T2
defines no peripheral global, so it does not depend on T1. T3 needs T1+T2 (the mock surfaces + globals
to compile against). T4 needs T1+T2+T3 (files must exist). T5a–T5e are independent of each other
(distinct files) and all need T3+T4 (harness + buildable targets). T6 edits distinct files (fixture
docstring + ADR) — no conflict with T5*. T7 is the verification gate after everything. No two tasks
edit the same file.

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
