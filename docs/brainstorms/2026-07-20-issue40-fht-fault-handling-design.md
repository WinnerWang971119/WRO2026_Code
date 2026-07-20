# Design: FHT — give every existing fault a non-human response

Brainstorm scratchpad: [`2026-07-20-issue40-fht-fault-handling.md`](./2026-07-20-issue40-fht-fault-handling.md)
Issue: [#40](https://github.com/WinnerWang971119/WRO2026_Code/issues/40)

## Problem

Every fault in the firmware currently terminates in the same place: `Mission` beeps five times,
enters `FAULT_AWAIT_RESET`, and waits for a human to press BTN_DOWN (`Mission.cpp:314-331`,
`:635-657`). On a scored WRO run there is no human — General Rules 10.7.2 ends the attempt if a
team member touches the robot. So a single fault, of any kind, costs the entire remaining run. On
top of that, `_enterFaultBeeping()` sets `_count = 0; _idx = 0` (`Mission.cpp:321-322`), destroying
the remaining mission queue, which means "skip the faulting step and carry on" is impossible by
construction today even if a human were available.

## Chosen direction

**Give each of the 17 faults that already exist a defined, non-human response.** No new fault
detection, no new watchdogs, no pose estimation, no new generated tables. The scope is exactly:
"handle every fault we already have."

The 17 existing fault kinds split cleanly along one axis — *had the robot moved when the fault
fired?*

### Class A — dispatch-time faults: pose intact → SKIP THE STEP

These fire from planning/dispatch guards that `return` before any `Drive` command is issued. The
robot is stationary on a known node and `Localization`'s pose is trustworthy. Verified by reading
each raise site.

| Fault | Raise sites (`PathTracker.cpp`) | Cause |
|---|---|---|
| `BAD_NODE` | `:673` | goal name did not resolve |
| `NOT_ZEROED` | `:678` | heading not bound before planning |
| `UNREACHABLE` | `:1989`, `:1996`, `:2886` | no `NEXT_HOP` route to the goal |
| `REVERSE_UNAVAILABLE` | `:2203`, `:2248`, `:2877` | reverse hop needs a rear board / not available |
| `PIVOT_DIRECTION_MISMATCH` | `:2155`, `:2316` | departure override contradicts approach direction (ADR-073) |
| `PIVOT_TRIM_UNDERFLOW` | `:3125` | seek into a pivot needs δ > sensor offset (ADR-044 D4) |

Six of the eleven `PathTracker::Fault` kinds. Every one is an authoring, geometry, or config error
caught *before* motion — and every one currently costs the whole run.

### Class B — in-motion faults: pose suspect → PARK

| Fault | Raise sites | Note |
|---|---|---|
| `DRIVE_FAULT` | `PathTracker.cpp:773`, `:877`, `:920`, `:1138` | generic guard + RETURN_TO_NODE + bounded-overrun |
| `OFF_NODE` | `:885`, `:1568` | operator-offset cap exceeded, `OFF_MAP_GATE == FAULT` |
| `LINE_LOST` | `:1009` | continuous loss ≥ `LINE_LOST_FAULT_MS` (#146) |
| `WALL_NOT_SQUARE` | `:1395` | **currently unreachable** — `N_RESET_NODES == 0` |
| `DRAIN_TIMEOUT` | `:726` | **sync-only path**; Mission always dispatches `.async()` so it never reaches this |

All five `DriveClass::Fault` kinds (`IMU_READ_FAILED` `Drive.cpp:1300`, `LINE_NOT_FOUND` `:1544`,
`:1582`, `:2060`, `DISPATCH_FAILED` `:2192`, `DRAIN_TIMEOUT` `:2261`) surface upward through
PathTracker's generic guard as `DRIVE_FAULT`, so they land in Class B too.

### The three changes this needs

1. **Stop destroying the mission queue.** `Mission.cpp:321-322` must preserve `_count`/`_idx` so a
   step can be skipped and the rest of the mission can run.
2. **A real terminal `PARK` state.** Brake, de-energize the motors, red LED, audible fault — and
   *stay there* without requiring a human. `FAULT_AWAIT_RESET` stays as the bench path (behind
   `MISSION_ALLOW_REARM`), but it stops being the only exit. This also banks every point already
   scored, per General Rules 10.8.
3. **A header-only `FaultResponsePolicy.h`** mapping fault kind → `{SKIP_STEP, PARK}`. Zero Arduino
   surface, so it links straight into the existing ctest harness alongside `TurnSignPolicy.h` /
   `StopAtLinePolicy.h` (ADR-030/049) and mirrors into `sim/runtime.py` for parity. This is the
   answer to "how do you validate a recovery layer with no hardware time."

## Rationale

**Why not the seed's own mechanisms.** The issue proposed "estimate position (Localization) to get
to the nearest reset point" and "try to find line." Both are dead on this stack:

- There is no in-hop pose estimate. `Localization::update()` (`Localization.cpp:359-390`)
  integrates the offset only while the nav gate is open, and on the closed→open edge it re-baselines
  from a single live read, explicitly discarding "any motion that happened while the gate was
  closed." `PathTracker::update()` (`:753`) holds that gate closed for the entire hop. At the instant
  a nav fault latches, `getOffsetFromNode()` reports the **pre-hop** offset.
- There is no reset point. `Routes::N_RESET_NODES == 0` (`routes.h:123`); the wall-zero interlude is
  wired and CI-tested but unreachable on the production field.
- "Try to find line" re-runs a primitive that just deterministically failed, and its *success* case
  is worse than its failure case: the seek latches whatever black line is under the sensor, which on
  a multi-line field may belong to a different edge, producing a confident wrong re-anchor.

**Why PARK rather than recover, for Class B.** The Critic and Feasibility agents converged
independently on a rules-level inversion: WRO scores the field's *final* state (General Rules 10.8),
so a robot that faults and freezes **keeps every point it already banked**. Meanwhile Senior Game
Rules §3.4 pays 7 × 4 = 28 points purely for barriers being "not damaged or moved," and those
barriers sit around the mosaic and cement target areas — exactly where a robot with an untrusted
pose would wander. Motion after a pose-loss fault therefore has negative expected value: it risks
banked points to chase remaining ones. Freezing is not a failure mode here, it is the
scoring-optimal action.

**Why SKIP is safe for Class A and only for Class A.** The pose that a skip leaves behind is the
pose the robot already had — it never moved. `PathTracker` replans from `Localization`'s actual
current node on the next step, so nothing downstream inherits a guess. That property is exactly what
Class B lacks, which is why the split is drawn where it is rather than per-fault by taste.

**Why no new watchdogs in this scope.** Four async paths still wedge with motors energized and raise
no fault (`Drive::_updateProfile` has no ceiling at all; `_updateStopAtLine` OVERSHOOT ignores its
own `stalled` flag; `_updateWallStop` BACK_OFF disarms the watchdog; Mission GOTO never reads the
`_stepStart_ms` it captures). Closing them means picking thresholds, and thresholds cannot be
honestly picked yet: `_updateProfile` derives progress from `EncoderSource.readDegreesOr(...)`, which
returns the *start* value on a transient lower-MCU read miss, so a jammed wheel, a sagging battery,
an I2C hiccup and a legitimately slow move all present identically. Phase 3's hardware gates
(`driveDist ±2 cm`, `turnDeg ±2°`, intersection ≥19/20) are unpassed and Lofaix is not brought up.
Deferred to national by decision, not oversight.

## Rejected / parked branches

Round 1:

- **Estimate position to reach the nearest reset point** (the seed's headline) — killed: no in-hop
  pose estimate and no reset node exists.
- **Try to find line** — parked as dangerous: retries a deterministic failure, and its success case
  (latching a different edge's line) is worse than its failure case.
- **Author the reset nodes that already exist** — deferred to national. The wall-zero recovery is a
  data gap, not a code gap, but authoring reset nodes needs the real field and a `wallStop` bench pass.
- **Pre-flight built-in test / go-no-go** — deferred. Cheap and independently valuable; refuses to arm
  when the gyro is unzeroed, a line board is offline (`isOnline()` exists and is unused as a gate), a
  servo is unhomed, or calibration is stale.
- **Observability reframe (post-run forensics)** — parked. Dump the `EventTrace` ring plus a per-hop
  residual log at fault entry and replay through CTPS; reducing fault *frequency* between attempts may
  beat recovering from faults during them.
- **Actuator FHT / end the silent-success lie** — deferred to national. `ManipBelt` forces "done" on
  timeout with only a `Serial.println` (`ManipBelt.h:415-425`), so Mission believes an actuation
  succeeded that never happened. The failure class most likely to produce *undetected* score loss.
- **Independent hardware watchdog (RA4M1 WDT)** — deferred. Recovers nothing but bounds the worst case.
- **Mission checkpoints with rollback**, **supervision tree over the three fault stores**,
  **kidnapped-robot relocalization by bounded search**, **analytical redundancy / which-sensor-lied**,
  **fault injection in sim + CTPS**, **make faults cheap by mission design** — all deferred to national.

Round 2 (the clock + ladder composition, explored before the narrowing):

- **Run clock as the supervisor** and the **degraded-mode ladder** — both deferred to national. The
  clock is still the right spine for a real supervisor (P6.6 remains unbuilt), and Fork A was resolved
  in favour of "two axes, sampled only at node arrivals" — the shape that dodges the pose-gate problem
  by construction, evaluating only where pose is known.
- **Freeze by default + offline safe corridor** — the chosen gating answer for national; its
  freeze-by-default half is what Class B's PARK implements now, minus the corridor.
- **Points ledger `EDGE_RISK[N][N]`**, **points-per-second budgeting**, **per-edge capability mask**,
  **offline `NEXT_HOP[L][N][N]` planes**, **confidence as a decaying scalar**, **Mission-DSL budgets**,
  **deadline registry below Mission**, **queue splice into a salvage script**, **explicit health probes**,
  **opening the Localization gate with an uncertainty envelope** — all deferred to national. Of these,
  the **deadline registry** (one `arm`/`disarm` mechanism both axes use, which also gives `Manipulator`
  a fault concept for free) is the strongest candidate to pick up first.

## Open questions

- **Does a skipped `goTo` need to skip its dependent steps?** Skipping the step leaves the robot on a
  node the mission did not plan for, so a later `task("place")` would actuate in the wrong place. The
  spec has to choose: skip only the faulting step, skip until the next step that re-establishes a
  known node, or let the operator declare dependencies. This is the one decision that changes the shape
  of the change.
- **Should consecutive skips escalate to PARK?** `NOT_ZEROED` in particular will re-fire on every
  subsequent `goTo`, so an unbounded skip policy walks the whole mission and parks anyway. A
  consecutive-skip cap would reach the same end state faster and more legibly.
- **Does `PARK` need to differ from today's `FAULT_BEEPING` terminal state** beyond not waiting for
  BTN_DOWN? Retracting the manipulator to a footprint-minimal pose and killing the belt were raised
  (branch #7, minimal-risk manoeuvre) but are unbuilt.
- **Where does the bench path live?** `FAULT_AWAIT_RESET` is still wanted on the bench.
  `MISSION_ALLOW_REARM` (default `true`, bench-only by intent) is the existing precedent for gating it.
