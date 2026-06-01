# ADR-027: `Drive.stopAtLine()` may read the line-sensor leaf; it re-anchors localization; arm-window rule

**Status:** Implemented
**Date decided:** 2026-05 (Phase 3, issue #27)

## Rule

`Drive` gains a stop-on-line motion primitive, `Drive.stopAtLine(expectedCm)`, a
`[[nodiscard]]` RAII fluent builder in the `driveDist` family. It drives the
chassis straight toward an expected perpendicular crossing line and stops on it
with an active brake. To detect the crossing, `Drive` reads the line sensor
directly — `LineArray.getJunctionType()` — a **read-only** dependency from the
Drive layer onto the `WROLineArray` perception leaf. `Drive.cpp` (not `Drive.h`)
holds the `#include "WROLineArray.h"`, so the sensor definition does not leak
into every translation unit that includes `Drive.h`.

Detection only **arms** once `traveled ≥ expectedCm − armWithin`
(`armWithin = Config::STOP_AT_LINE_ARM_WINDOW_CM`, overridable per call): a line
crossed earlier en route cannot stop the chassis. The arm threshold clamps to
`0` when `armWithin ≥ expectedCm` (small-hop case — armed from the first tick).
If no `T_OR_CROSS` is seen by `expectedCm + Config::STOP_AT_LINE_MAX_MARGIN_CM`,
Drive fail-fast faults `LINE_NOT_FOUND` (stop + clear queue). A no-progress stall
watchdog (`Config::STOP_AT_LINE_STALL_TIMEOUT_MS`) also faults `LINE_NOT_FOUND`
if the encoder reports no advance for that interval, closing the async-FSM
infinite-hang hole. There is no silent dead-reckon fallback.

When a `PathTracker` blind hop resolves to stop-at-line, success advances
`Localization` via `onIntersection()` (a true node re-anchor), where a plain
blind hop would use `onDeadReckonedAdvance()`. Re-anchor here corrects the
discrete `_currentNode` id only; heading stays IMU-live by design (ADR-005).

## Why

Issue #27 asked for a `stopAtLine()` verb and for the path planner to use it for
"better localization." Dead-reckoned blind hops accumulate encoder drift (Risk #9);
stopping on a physically-taped crossing and re-anchoring there is the cheapest way
to bleed off that drift without odometry or SLAM.

The verb lives on `Drive`, not `LineFollower`, because the user chose this
explicitly after being shown the tradeoff: it matches the issue title and the
`driveDist` fluent-builder family, and the seek is a `Drive`-mode (SYNC/GYRO
bounded-motion) concept. The strongest rejected alternative was `LineFollower`
(it already holds the `WROLineArray` pointer, a distance gate, and rear-board
selection, so it would add *zero* new dependency edges) — rejected because
`LineFollower` would have to proxy a Drive-mode move it does not own. The trade
accepted is one new cross-layer read edge (`Drive → WROLineArray`), which this
ADR records. `WROLineArray` is a leaf sensor, so the graph stays acyclic.

The arm window is load-bearing and counter-intuitive: it guards against *early*
(already-passed) lines, but it only fires on the true line if the encoder
estimate of `traveled` is within `armWithin` of reality. If dead-reckoning
*under-travels* by more than `armWithin`, the chassis crosses the real line while
still disarmed and misses it (runs to max → fault). So `armWithin` MUST be sized
larger than the worst-case under-travel error over the hop — exactly the drift
this feature exists to correct, so the window is generous, not tight.

## How to apply

- **Bare verb:** `Drive.stopAtLine(40);` drives, detects, axle-overshoots, and
  active-brakes (sync-blocking). Knobs follow ADR-012: `.setMode(...)`
  (default `SYNC`), `.setPower/.setPreset`, `.setHeading`, `.setArmWithin`,
  `.setMaxDistance`, `.setOvershoot`, `.setReverse`, `.setBrake`, `.async()`.
  `Drive.isLineFound()` latches the last result; no-line latches
  `Fault::LINE_NOT_FOUND`.
- **Detection** reads `LineArray.getJunctionType() == T_OR_CROSS` (named constant
  `WROLineArrayClass::JUNCTION_T_OR_CROSS`) directly, with a
  `Config::STOP_AT_LINE_DETECT_TICKS` rising-edge debounce. Drive does **NOT**
  consume `Intersection.isAtIntersection()` — that bit is read-and-clear and owned
  by PathTracker's nav consumer; reading it would steal the nav arming edge.
  PathTracker drains `Intersection.isAtIntersection()` at stop-at-line dispatch
  time so the seek's physical crossing cannot leak a stale nav edge into the next
  LINE hop.
- **Map trigger:** author a BLIND edge with `blind_mode: STOP_AT_LINE` in
  `field_data`; the pipeline emits `BlindMode::STOP_AT_LINE` (value `4`).
  **Call trigger:** tri-state `PathTracker.goTo(name).setStopAtLine(StopAtLine::ON|OFF|AUTO)`
  (and `Mission.setStopAtLine(...)`); the call-site ON/OFF wins over the edge,
  `AUTO` defers to the edge (mirrors `arrival_direction` precedence, ADR-024).
- **Unified resolver:** the ON/OFF/AUTO precedence rule and board-availability
  check live in `WRO2026/StopAtLinePolicy.h::resolveStopAtLine` (header-only,
  mirroring `DirectionPolicy.h`). The normative source is the truth-table fixture
  `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv`; the Python
  twin is `sim/runtime.py::resolve_stop_at_line`. TC13 enforces cross-language
  parity (same gate as `resolveDirection`/TC18). The resolver replaced the inlined
  PathTracker switch and the host-test `replica_resolveStopAtLine`.
- **Seek drive mode:** the seek defaults to `DriveMode::SYNC`. A call-site
  `setStopAtLine(ON)` forced onto a typed blind edge (`GYRO_TSHAPE`, `RAW_FLAT`)
  drives the seek in that edge's mode, with the resolved body heading passed to
  `setHeading(...)` for `GYRO`. Edges typed `STOP_AT_LINE`/`SYNC_TSHAPE`/`DEFAULT`
  resolve to `SYNC`.
- **Reverse** (`.setReverse(true)`) is a first-class, supported path on a 2-board
  chassis: the rear board detects the crossing; the compiler and Python sim emit
  `ACT_STOP_AT_LINE_REVERSE`; `Localization.onIntersection()` re-anchors on
  success. Logic is CI-tested (host C++ + Python parity via TC13); physical
  rear-board bench (B3.7-reverse) is deferred until the rear board is wired. On a
  1-board chassis a forced-REAR stop-at-line hop degrades to a reverse
  dead-reckoned blind move (no fault, no 180°-turn): the resolver returns
  `seek=false` and PathTracker emits the dead-reckon path instead. Drive's bare
  reverse-reject remains as defense-in-depth (now unreachable from PathTracker,
  which degrades first).

## Seek state machine

The `_updateStopAtLine` FSM runs four phases. All transitions are polled each
cooperative tick; no blocking.

**SEEK** — drive blind toward the endpoint. The line detector is **disarmed**; any
line crossed early is ignored. Arms once `traveled >= expectedCm - armWithin`
(clamps to 0 for small hops, so armed from tick 1). Faults `LINE_NOT_FOUND` if
`stopAtLineSeekMaxedOut(traveled, maxCm)` or `stopAtLineStalled(...)` trips.

**ARMED** — continue cruising while watching for a debounced `T_OR_CROSS`
(`Config::STOP_AT_LINE_DETECT_TICKS` consecutive reads). Also checks
`stopAtLineSeekMaxedOut` and the stall watchdog each tick. Transitions to
OVERSHOOT on the debounced trigger.

**OVERSHOOT** — rolls the chassis by the known axle-to-sensor offset so the wheel
axle lands on the line, ramping to the deadband floor. No detector activity during
this phase. Transitions to DONE when the overshoot distance is covered.

**DONE** — issues a final active-brake, restores the front board as the active
line-sensing board, and calls `_endStopAtLine(true)`. PathTracker receives the
idle signal and calls `Localization.onIntersection()`.

**Fault exits** (from SEEK or ARMED): `LINE_NOT_FOUND` is latched if
`stopAtLineSeekMaxedOut(traveled, maxCm)` (max-distance overshoot) OR
`stopAtLineStalled(nowMs, lastProgressMs, Config::STOP_AT_LINE_STALL_TIMEOUT_MS)`
(no encoder advance for the stall window). Both predicates live in `WROMath.h`
as pure inline functions — host-testable without hardware.

## Trigger to revisit

- Opportunistic mid-blind re-anchoring (watching for unexpected lines *outside*
  the arm window) is wanted — explicitly rejected here for being fuzzy; it would
  reopen the arm-window contract.
- A perception module needs to correct yaw/xy (not just the node id) at a
  stop-at-line — that widens the `Localization` write set (ADR-005 trigger).
- `STOP_AT_LINE` is wanted on `LINE` or `PROJECTION` edges — out of scope here
  (a segment ending at a crossing is already a `LINE` edge); revisit if the edge
  taxonomy changes.
- Physical B3.7-reverse bench pass once the rear board is wired.

## See also

- [ADR-019 — Custom 8-channel `WROLineArray` (the perception leaf `Drive` now reads)](./019-8channel-custom-line-sensor.md)
- [ADR-005 — `Localization` single source of truth (re-anchor updates the node id only)](./005-localization-single-source-of-truth.md)
- [ADR-024 — Bidirectional line following (rear-board reverse + 3-layer override precedence)](./024-bidirectional-line-following.md)
- [ADR-012 — Fluent builder for verbs with 3+ knobs](./012-fluent-builder-3plus-knobs.md)
- [ADR-016 — Drive builders block by default; `.async()` opts out](./016-sync-default-builders.md)
- [ADR-017 — `DriveMode::SYNC` + `tShape` are the project-wide default](./017-sync-plus-tshape-default-profile.md)
- `WRO2026/StopAtLinePolicy.h` — the single C++ source of the resolver.
- `docs/plans/2026-05-30-stop-at-line.md` — the originating spec.
- `docs/plans/2026-06-02-stop-at-line-option-b-fixes.md` — Option B + review-findings fix plan.
