# Phase 2 PR2 — Manual C++ Compile Audit

**Date:** 2026-05-06
**Reason:** `arduino-cli` not installed on Diego's Windows machine; per AC6 of
`docs/plans/2026-05-06-phase2-pr2-path-tracker-runtime.md`, fall back to a manual review log
listing every file touched, every public method exposed, every declared-but-undefined trap
checked, and every cross-module include + extern that must resolve at link time.

This is **not** a substitute for an actual compile. The next bench session must run
`arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` (or the IDE Verify) and
record the output. Any errors found there are PR 2.5 patches.

---

## Files touched

| File | Purpose | Status |
|------|---------|--------|
| `WRO2026/Config.h` | Added `PATH_QUEUE_CAPACITY`, `PATH_DRAIN_TIMEOUT_MS_PER_GOAL`, `NAV_TURN_TOLERANCE_DEG` | Edited |
| `WRO2026/FieldMap.h` | Header-only namespace over `Routes::` | New |
| `WRO2026/Localization.h` | Singleton declaring 6 public methods + extern instance | New |
| `WRO2026/Localization.cpp` | Implementation of the 6 methods + `_normalize` | New |
| `WRO2026/LineFollower.h` | Added `setDistanceTarget(cm)` builder setter, `isDistanceReached()` getter, distance-gate state | Edited |
| `WRO2026/LineFollower.cpp` | Builder setter impl, `_applyEnableBuilder` signature change, distance gate inside `update()`, `isDistanceReached()` impl | Edited |
| `WRO2026/PathTracker.h` | Builder + class declarations, FSM enum, fault enum + struct, queue, friend declaration | New |
| `WRO2026/PathTracker.cpp` | FSM tick, queue plumbing, drain spin, builder destructor, goTo overloads | New |
| `WRO2026/WRO2026.ino` | Replaced `DEMO_MODE` with `NAV_DEMO_MODE`; added Localization + PathTracker to setup + cooperativeTick; tick callbacks | Edited |
| `WRO2026/routes.h` + `routes.cpp` | Schema bump: `goal_node` field added to `GoalApproach`. Auto-regenerated. | Auto-generated |
| `tools/path_planner/header_writer.py` | Emit `goal_node` field; KeyError on missing goal node | Edited |
| `tools/path_planner/tests/golden/routes_toy.h` + `.cpp` | Updated golden fixture | Auto-regenerated |
| `tools/path_planner/sim/__init__.py` + `chassis.py` + `runtime.py` | Pure-Python end-to-end simulator | New |
| `tools/path_planner/tests/test_runtime_sim.py` | 11 tests covering TC1–TC7, TC10–TC11, plus tolerance regression and walk_next_hop self-target | New |
| `docs/PHASE2_DEFERRED.md` | Graduate simulator entry | Edited |
| `docs/PATH_PLANNER_SPEC.md` | One-line note in §5.1 about float-degree heading | Edited |

---

## Per-module review

### `WRO2026/Config.h` (edit)

- New constants: `PATH_QUEUE_CAPACITY` (uint8_t, 8), `PATH_DRAIN_TIMEOUT_MS_PER_GOAL`
  (uint32_t, 60000), `NAV_TURN_TOLERANCE_DEG` (float, 0.5f). All `constexpr`. Doxygen on each.
- No magic numbers introduced.
- // reviewed manually for: declared-but-undefined / extern mismatch / missing include — N/A
  (header-only constant declarations).

### `WRO2026/FieldMap.h` (new)

- Header-only namespace `FieldMap` with 7 inline functions: `getNextHop`, `getEdge`,
  `getEdgeType`, `getNode`, `getNodeType`, `getGoalApproach`, `findNodeByName`.
- 3 `constexpr` re-exports: `N_NODES`, `N_GOALS`, `NODE_NONE`.
- Includes `<stdint.h>`, `<string.h>`, `"routes.h"`.
- // reviewed manually for: declared-but-undefined / extern mismatch / missing include —
  every function is `inline` with a body present in this header. `Routes::` symbols are all
  declared in `routes.h` (re-checked: `NEXT_HOP`, `EDGES`, `NODES`, `GOAL_APPROACH`,
  `N_NODES`, `N_GOALS`, `NEXT_HOP_NONE` all present).

### `WRO2026/Localization.h` + `.cpp` (new)

- Public surface: `begin()`, `zeroHeadingFromCurrent()`, `isZeroed()`, `getCurrentNode()`,
  `getHeadingDeg()`, `toImuYaw(float)`, `onIntersection(uint8_t)`,
  `onDeadReckonedAdvance(uint8_t)`. Each declared in `.h`, defined in `.cpp`.
- Private: `_currentNode`, `_yawOffsetDeg`, `_zeroed`, `static _normalize`.
- `extern LocalizationClass Localization;` in `.h`; instance defined in `.cpp` (line 12).
- Includes: `<Arduino.h>`, `<stdint.h>`, `"FieldMap.h"`, `"routes.h"` in header. `.cpp` adds
  `<MatrixMiniR4.h>` for `MiniR4.Motion.getEuler` and `<math.h>` for `fmodf`/`lroundf`.
- // reviewed manually for: declared-but-undefined / extern mismatch / missing include —
  every header method has a matching `.cpp` definition. `Routes::START_NODE` and
  `Routes::START_HEADING_DEG` both exist in `routes.h`.

### `WRO2026/LineFollower.h` + `.cpp` (edit)

- New public method on `LineFollowerClass`: `bool isDistanceReached()` (read-and-clear).
- New builder setter: `LineFollowerEnableBuilder& setDistanceTarget(float cm)`.
- `_applyEnableBuilder` signature gained two new trailing parameters
  (`hasDistanceTarget`, `distanceTargetCm`). Both call sites updated:
  1. Builder destructor (`LineFollower.cpp` line ~28).
  2. The distance-gate self-disable inside `update()` (`LineFollower.cpp` end-of-update).
- New private members: `_distanceTargetActive`, `_distanceTargetCm`, `_distanceStartEncL`,
  `_distanceStartEncR`, `_distanceReached`. All zero-initialized.
- // reviewed manually for: declared-but-undefined / extern mismatch / missing include —
  the new builder setter is declared in `LineFollower.h` and defined in `LineFollower.cpp`.
  The new public `isDistanceReached` is the same. `Drive.getEncL/R()` (used in
  `_applyEnableBuilder` and `update`) are public members of `Drive` per `Drive.h:401-408`.
  `encDegToCm` and `presetToPwm` are free functions in `WROMath.h`, which `LineFollower.cpp`
  inherits via `LineFollower.h` -> `Config.h`. Already used in the existing tShape branch.

### `WRO2026/PathTracker.h` + `.cpp` (new)

- Public surface on `PathTrackerClass`: `begin()`, `update()`, `setTickCallback(void(*)())`,
  `isIdle()`, `goTo(uint8_t)`, `goTo(const char*)`, `hasFault()`, `getFault()`, `clearFault()`,
  plus the friend-accessible `_applyGoToBuilder` (private but `friend class PathTrackerGoToBuilder`).
- Public surface on `PathTrackerGoToBuilder`: ctor `(PathTrackerClass&, uint8_t)`, dtor,
  `async()`. Copy/move deleted.
- Forward declaration of `class PathTrackerClass;` at top of `.h` so the builder can name
  the type. `_applyGoToBuilder` body is in `.cpp` where the full class is visible.
- `extern PathTrackerClass PathTracker;` in `.h`; instance defined in `.cpp` (line 14).
- Includes: `.h` uses `<Arduino.h>`, `<stdint.h>`, `"Config.h"`, `"FieldMap.h"`, `"routes.h"`.
  `.cpp` adds `<math.h>`, `"Drive.h"`, `"Intersection.h"`, `"LineFollower.h"`, `"Localization.h"`.
- // reviewed manually for: declared-but-undefined / extern mismatch / missing include —
  every header method has a matching `.cpp` definition (audited via grep on method names).
  Every Drive/LineFollower/Intersection/Localization/FieldMap call resolves to a public
  method in those headers.
- // reviewed manually for: AC8 (no sync verbs from update()) — `grep "Drive\." PathTracker.cpp`
  shows: 3x `Drive.stop(true)` (all in fault/safety paths, not update() chain), 4x
  `Drive.isIdle()` (poll), 1x `Drive.hasFault()` (poll), and 5x motion verbs all chained
  with `.async()`: `Drive.turnToYaw(...).async()` (TURNING entry, GOAL_BLIND TURN_TO_HEADING),
  `Drive.turnDeg(...).async()` (GOAL_BLIND TURN_RELATIVE), `Drive.driveDist(...).async()`
  (BLIND_FORWARD, GOAL_BLIND FORWARD). LineFollower calls are all builder-pattern (apply
  state synchronously inline) or read-only.
- // reviewed manually for: builder destructor double-execute — `_executed = true` set after
  the apply call; copy/move deleted at compile time.

### `WRO2026/WRO2026.ino` (edit)

- Includes added: `"FieldMap.h"`, `"Localization.h"`, `"PathTracker.h"`.
- `cooperativeTick()` order: Drive -> LineFollower -> Intersection -> Manipulator -> PathTracker
  -> EventTrace. PathTracker after Intersection so its read-and-clear edge sees this tick's
  event (AC9).
- `setup()` extended: `Localization.begin()` and `PathTracker.begin()` after Manipulator;
  `PathTracker.setTickCallback(cooperativeTick)` after `Drive.setTickCallback`. New `locOk` /
  `pathOk` join the `_setupFailed` accumulator.
- `loop()` body: `cooperativeTick()` always; `NAV_DEMO_MODE` branch replaced `DEMO_MODE`
  entirely. BTN_UP rising edge calls `Localization.zeroHeadingFromCurrent()` then chains
  `PathTracker.goTo("C"); PathTracker.goTo("G");` with explicit `hasFault()` checks between.
- // reviewed manually for: declared-but-undefined / extern mismatch / missing include —
  all referenced symbols are public methods of their respective singletons, all headers are
  included.

---

## AC checklist (recap)

- [x] AC1: `uv run python tools/path_planner/build_routes.py` succeeds with new schema.
      Confirmed manually 2026-05-06.
- [x] AC2: `uv run pytest tools/path_planner/tests/ -v` passes (83 tests, 1.44s).
- [x] AC3: `PathTrackerSim` exposed; `test_runtime_sim.py` covers TC1, TC2, TC3, TC6 — all
      pass.
- [x] AC4: chassis arrival within 0.1 cm verified by
      `test_every_reachable_goal_lands_within_tolerance` (passes).
- [x] AC5: `UnreachableGoalError` raised by `test_unreachable_goal_raises` (passes).
- [ ] AC6: arduino-cli not installed. **This file is the fallback per the AC.**
- [x] AC7: `LocalizationClass::zeroHeadingFromCurrent()` is the only writer of `_yawOffsetDeg`
      and `_zeroed` (verified via grep). Python mirror's NOT_ZEROED behaviour is asserted by
      `test_goto_before_zero_raises`.
- [x] AC8: see PathTracker.cpp audit above. No sync verbs from update().
- [x] AC9: see WRO2026.ino audit above.
- [x] AC10: `docs/PHASE2_DEFERRED.md` graduates the simulator entry; wall_zero, calibration
      nodes, per-scenario tables, Serial console all remain.
- [ ] AC11: branch + commits — to be done in T12 commit step.
- [x] AC12: only files listed above were modified. No PR1 Python files changed except
      `header_writer.py` (schema-bump touch) and the golden fixture (auto-regenerated).

---

## Known limitations & PR 2.5 hardware-bring-up TODO

- `Drive.turnToYaw(...).async()` may block the cooperative tick if the lower-MCU firmware
  silently ignores the async flag. Spec section "Notes for Implementer" carries the
  contingency note. First bench session is the test.
- `LineFollower::setDistanceTarget` uses `(|encL_delta| + |encR_delta|) / 2` to compute
  travelled cm. On a tight curve where one wheel travels meaningfully more than the other,
  this may trip the gate slightly early or late. Acceptable for the toy graph (straight
  segments) but a candidate for revisit on the real field.
- The Phase 1 `DEMO_MODE` smoke is GONE in this PR — the `NAV_DEMO_MODE` block exercises
  every Drive/LineFollower/Intersection primitive end-to-end via PathTracker. If a Phase 1
  smoke is wanted again later, it can be re-added as a third `#ifdef`.
