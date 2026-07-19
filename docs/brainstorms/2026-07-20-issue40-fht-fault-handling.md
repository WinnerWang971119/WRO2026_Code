# Brainstorm: FHT — Fault Handle Technology (issue #40)

**Seed (verbatim):**
> # Idea
> The idea is to solve the problem:
> Yes, the robot fault at comp, but it stop moving.
>
> If we can have a tech that let it can resume to compt such as:
> Skip the task that is faulting
> Try to findline.
> Try to estimate position(Locolization) to get to the neareast reset point.
> Or so. Need brainstorming

Issue #40 — labels: `enhancement`, `P1: high`, `area: nav`, `needs-design`. Open.

**Context summary (Phase 1 findings):**

_Prerequisites from the issue's own comment — current status:_
- #145 (latched Drive fault survives reset/abort) — **CLOSED**. `PathTracker::clearFault()` / `abort()` now cascade `Drive.clearFault()` (`PathTracker.cpp:530`, `:580`); `Mission::abort()` also clears directly (`Mission.cpp:219`). ADR-067.
- #146 (lost line wedges silently) — **CLOSED**. `PathTracker::Fault::LINE_LOST` fires after `Config::LINE_LOST_FAULT_MS` (500 ms) of continuous loss in `LINE_FOLLOW` (`PathTracker.cpp:977-1011`), suppressed during deliberate pass-through windows. A second bounded-overrun watchdog routes `ApproachPhase::OVERRUN` → `DRIVE_FAULT` (`PathTracker.cpp:1121-1139`).

_Watchdog gaps still OPEN (a fault never fires → nothing for FHT to react to):_
- `Drive::_updateProfile` — no elapsed ceiling, no no-progress check (`Drive.cpp:782-870`). Every `.async()` `driveDist(...).tShape()` (all BLIND hops, WALL_DEPART, dead-reckon return) can wedge with motors energized.
- `_updateStopAtLine` OVERSHOOT — `stalled` computed (`Drive.cpp:1371`) but only SEEK/ARMED consult it; OVERSHOOT body (`:1490-1529`) ignores it.
- `_updateWallStop` BACK_OFF — watchdog explicitly disarmed (`Drive.cpp:1709`); stall/timeout are PUSH-scoped only.
- Mission GOTO — `_stepStart_ms` captured (`Mission.cpp:423`) but never compared. `PATH_DRAIN_TIMEOUT_MS_PER_GOAL` (60 s) only applies to *sync* `goTo`, and Mission always dispatches `.async()` (`Mission.cpp:417-421`).
- `Manipulator` / `ManipBelt` / `ManipServo` — **no fault enum, no `hasFault()` at all.** Belt/servo timeouts force "done" with a `Serial.println` and no queryable state (`Manipulator.cpp:264-296`, `ManipBelt.h:415-425`).

_Recovery that exists today:_
- Fault → `_enterFaultBeeping()` (stop, red LED, 5 beeps) → `FAULT_AWAIT_RESET` → **wait for a human BTN_DOWN** (`Mission.cpp:314-331`, `:596-657`). That is the entire recovery path at the Mission layer. On a scored run there is no human.
- `Mission::declarePosition(name)` — operator-typed re-anchor + fault clear (`Mission.cpp:230-285`). Also human-gated.
- `PathTracker` RETURN_TO_NODE — the one **autonomous** recovery today: offset > `RETURN_TRIGGER_THRESHOLD_CM` (3 cm) and ≤ `RETURN_DISTANCE_CAP_CM` (50 cm) → drive the inverse-offset move back, re-anchor via `stopAtLine` seek or dead-reckon (`PathTracker.cpp:832-933`, `ReturnToNodePolicy.h:79-102`). Beyond the cap: `OFF_MAP_GATE == FAULT` → `Fault::OFF_NODE`.
- Wall-zero reset interlude (`WALL_STOP` → `REANCHOR`, `PathTracker.cpp:1329-1398`) is fully wired but **currently unreachable: `Routes::N_RESET_NODES == 0`** — no reset node is authored in the field model.
- `Tasks`: `abort()` sets a sticky `_aborted` that "subsystems do not yet poll". Mission *already* skips a task that returns `false` without a PathTracker fault (`Mission.cpp:573-585`).
- **No 2-minute run clock / time guard exists** (Roadmap P6.6, pending).

_Roadmap position:_ Phase 3 (hardware verification) is "next"; recoveries are Phase 6. Regional `2026-07-21`, national `2026-08-24`, worlds `2026-11-30`.

**Clarified framing:** _(filled after Phase 2)_

## Perspectives (round N)
### Critic
### Feasibility
### Thinker (branches)

## Live direction

## Killed / parked

## Decisions & debate verdicts
