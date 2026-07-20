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

**Clarified framing:**

FHT is the layer that turns "the robot faulted, so it stopped" into "the robot faulted, and kept
scoring." Today every Mission-layer fault terminates in `FAULT_AWAIT_RESET` waiting on a human
BTN_DOWN, which on a scored WRO run is not available — so a single fault costs the whole
remaining run. **Diego's chosen target deadline is the regional, `2026-07-21` — tomorrow**, which
means tier-0 FHT must be something flashable in hours, on top of a stack that is supposed to be
frozen, and must be net-safe: a watchdog that false-fires mid-run is strictly worse than a wedge.
The desired end-states are two, in priority order: (1) **recover in place and continue the
mission** where the situation allows, and (2) **safe-park and preserve points already scored**
when it doesn't — never wander into placed tiles or off the mat. The failure class to attack
first is **nav/pose loss** (lost line, missed junction, blind-hop drift, OFF_NODE), with the
broader failure taxonomy explicitly open for exploration — Diego is not yet certain which class
will actually bite. Constraints that bound every option: single-threaded cooperative `loop()`, no
heap after `setup()`, no `delay()` in `update()` paths, `Localization` is single-writer,
`N_RESET_NODES == 0` today (no wall-zero anchor exists to drive to), no run clock exists, and
`Manipulator` has no fault concept at all.

## Verified-by-hand facts (checked in source during this session)

- `MissionClass::_enterFaultBeeping()` sets `_count = 0; _idx = 0` (`Mission.cpp:321-322`) — **the
  remaining mission queue is destroyed at fault entry.** "Skip the faulting step and continue" is
  impossible by construction today; even a free auto-ack would resume into an empty queue.
- `Localization::update()` (`Localization.cpp:359-390`) integrates the offset only while the nav
  gate is OPEN, and on the closed→open edge it re-baselines from a single live read —
  "discards any motion that happened while the gate was closed." `PathTracker::update()` holds
  that gate closed for the entire hop (`PathTracker.cpp:753`). **At the instant a nav fault
  latches, `getOffsetFromNode()` reports the PRE-HOP offset.** There is no dead-reckoned in-hop
  displacement to estimate a position from.
- `Routes::N_RESET_NODES == 0` (`routes.h:123`). Only the host-test fixture has 2. **No reset
  point exists to drive to.**
- `WRO2026/MissionScript.cpp` is 16 lines: one step, `.task("testOne")`. **There is no authored
  competition mission for FHT to protect.**
- Working tree is mid-tune on `test/tune-robot-third-time` with uncommitted `ConfigDrive.h` /
  `ConfigManipulator.h`. Nothing is frozen.
- Build headroom is NOT a constraint: flash 174 256 / 262 144 B (66 %), RAM 18 928 / 32 768 B
  (57 %) — verified by compiling.

## Perspectives (round 1)

### Critic

**Problems (strongest first):**

1. **The seed's headline mechanism — "estimate position (Localization) to get to the nearest reset
   point" — cannot be built on this stack, because neither the estimate nor the reset point
   exists.** `PathTracker.cpp:753` closes the Localization integrate gate for the entire duration of
   a hop, and `Localization::update()` (`Localization.cpp:363-381`) not only refuses to integrate
   while the gate is closed, it *deliberately discards* all closed-period motion by re-baselining
   from a single live read on the reopen edge. So at the exact instant a nav fault latches,
   `getOffsetFromNode()` returns ~the pre-hop offset — the robot believes it is standing on the node
   it left, no matter how far down the edge it actually travelled. And the destination is equally
   absent: `routes.h:123` is `N_RESET_NODES = 0`. FHT tier-0 would have to invent both a pose
   estimator and a reset-node authoring pass, on the day before the regional.

2. **Under the actual WRO 2026 rules, "keep moving after a fault" has negative expected value,
   while "stop and freeze" is the scoring-optimal behavior the code already implements by
   accident.** Scoring is evaluated on the field's *final* state, and the Senior rules pay
   7 points × 4 = **28 points purely for barriers being "not damaged or moved"** (Senior §3.4),
   where "moved" is any part touching the mat outside its grey area — and those barriers sit in the
   middle around the mosaic and cement target areas, i.e. squarely where a lost robot would wander.
   Already-placed tiles and delivered cement are scored on final position too. General Rules 10.7.3
   ends the attempt outright if the robot "has completely left the game table," and 10.7.5 means a
   team shouting STOP only ends the run *once the robot stops by itself* — you cannot halt a runaway
   without touching it (10.7.2, which also ends the run). The current wedge preserves the score;
   FHT is a mechanism for spending banked points on a lottery ticket.

3. **Tier-0 FHT would land untested-on-hardware code into the highest-blast-radius modules, hours
   before a competition, on a stack that is not frozen and has never passed its own verification
   gates.** Drive constants were retuned today (`6627bb3`), with `ConfigDrive.h` and
   `ConfigManipulator.h` still uncommitted. Roadmap Phase 3 is still *next*; P6.8 (ten-run scoring
   battery) has never run, so nobody knows the real fault distribution FHT is meant to handle. The
   recovery layer must sit inside `Drive`, `PathTracker`, `Mission` — the three modules the whole
   run depends on. A false-firing watchdog is not a degraded FHT; it is a regression on the healthy
   path.

4. **The watchdog gaps FHT must close first are precisely the ones where a sensor failure and a
   physical stall are indistinguishable, so any ceiling written tonight is an untuned guess.**
   `_updateProfile` (`Drive.cpp:782-870`) computes progress from
   `EncoderSource.readDegreesOr(MOTOR_LEFT_ID, _profileStartEnc)` — on a transient lower-MCU read
   miss it *deliberately* returns the start value, i.e. "zero travel this tick." A jammed wheel, a
   sagging battery, an I2C hiccup, and a legitimately slow move all present as the same `frac`
   stagnation. Picking an epsilon needs per-move timing from the real chassis; Lofaix (EV3 gearing,
   different torque and encoder scaling than the TT rover the constants were tuned on) is not
   brought up. Same for OVERSHOOT (`stalled` computed at `:1371`, never consulted at `:1490-1529`)
   and BACK_OFF (watchdog *explicitly disarmed* with a justifying comment at `:1709`) — every one is
   a deliberate prior decision being reversed without data.

5. **"Skip the faulting task" is unsound because the actuator layer cannot report failure, so
   Mission cannot tell a failed task from a successful one — and skipping a nav step corrupts the
   pose every subsequent step depends on.** `Manipulator::_drain` (`Manipulator.cpp:264-296`)
   returns on timeout with a `Serial.println` and no fault; `ManipBelt` forces `_state = IDLE` on
   stall (`ManipBelt.h:415-425`) — "the belt reports done either way." Mission's existing skip path
   (`Mission.cpp:573-585`) fires on a task that *returned false*, the one case that already works;
   the dangerous case — a task that silently didn't actuate — is invisible. Worse, skipping a `goTo`
   leaves the chassis somewhere the next `goTo` doesn't expect, and `Localization`'s single-writer
   pose (ADR-005) has no confidence field — every consumer trusts a guessed anchor as absolute
   truth. A robot carrying an ungrabbed tile it later drops mid-field scores zero *and* triggers the
   "moved" penalties in problem 2.

6. **"Try to find line" re-runs the exact primitive that just failed, with the same inputs, from a
   pose that is now unknown — and its success case is more dangerous than its failure case.**
   `Fault::LINE_NOT_FOUND` *is* what the seek raises when it cannot find a line; `Fault::LINE_LOST`
   after `LINE_LOST_FAULT_MS = 500` (`ConfigNav.h:74`, marked "TODO bench-tune") is what LINE_FOLLOW
   raises. Re-seeking is retrying a deterministic failure. And if the seek *does* latch, it latches
   onto whatever black line is under the sensor — on a mosaic field with many lines, possibly a
   different edge entirely. That produces a confident, wrong re-anchor, after which the robot drives
   a fully-planned route at cruise speed through a mis-registered field. `OFF_MAP_GATE == FAULT`
   (`ConfigNav.h:395`) is exactly what prevents this today; FHT's natural implementation is to flip
   it to `BEST_EFFORT`, which the existing code already treats as the risky branch.

7. **FHT is being specified against an imagined fault distribution, and it crowds out two cheaper
   interventions that dominate it on expected points.** No run has ever been scored on the real
   field — `field_data.json` is still the test map, P6.8 has not been executed — so there is zero
   evidence about which of the 12 `PathTracker::Fault` or 5 `DriveClass::Fault` values actually
   fires on hardware. Two things strictly dominate and are hours-scale: (a) a hard motor-kill on the
   unwatched async paths — a *stop*, not a recovery, converting "wedged with motors energized,
   grinding into a barrier" into "frozen, points banked"; and (b) the 2-minute run clock with a
   forced return (P6.6), which does not exist anywhere. Neither needs a pose estimator, a
   reset-node pass, or trusting a guessed anchor.

_Sources: WRO 2026 RoboMission General Rules §10.7.2/10.7.3/10.7.5, §10.10–10.11, §4.d; WRO 2026
RoboMission Senior Game Rules "Mosaic Masters" §3.4 barrier bonus, §4 scoring sheet._

### Feasibility

**VERDICT: Conditional**

**Must-be-true conditions**
- **The scoring model must reward not-stopping.** It does, weakly. General Rules 10.7.2 ends the
  attempt if any team member touches the robot, and 5.2.6 makes the start button also the stop
  button — human rescue is genuinely unavailable, confirming the premise. But 10.8 scores "the
  situation on the field at this point of time", so a robot that faults and stops **keeps every
  point it already banked**. Tier-0 therefore buys (i) remaining points and (ii) a lower recorded
  time (10.13 tiebreak, via 10.7.5 "shout STOP") — not the preservation of banked points, which are
  already safe today.
- **Any FHT action must be pose-safe.** Nav/pose loss is exactly the class where the firmware's
  belief is untrustworthy. "Skip and continue" dispatches the next `goTo` from
  `Localization.getCurrentNode()` — the value the fault just declared unreliable. It is only
  net-safe for *task* faults (manipulator/vision), where pose is intact.
- **Thresholds must be measured, not guessed.** Phase 3's gates are unpassed: `driveDist ±2 cm`,
  `turnDeg ±2°`, intersection ≥19/20 all unverified. A guessed timeout is precisely the
  "false-firing watchdog is worse than a wedge" case.
- **Resource budget must fit.** It does — verified by compiling: flash 66 %, RAM 57 %, ~13.8 KB
  free. A few hundred bytes of FHT state is not a constraint.

**Hard blockers**
- **No anchor mechanism exists on the real field, so "recover in place and continue" is impossible
  in tier-0.** The wall-zero interlude (`PathTracker.cpp:1329-1398`) is wired but unreachable:
  `routes.h:123` `N_RESET_NODES = 0`; only `test/fixtures/routes_fixture.h:123` has 2. Fixing it
  means authoring reset nodes in `field_data.json`, regenerating routes, and bench-verifying
  `wallStop` — not an hours-scale change.
- **The rear line board is not wired, but `Config::LINE_BOARD_COUNT` already defaults to `2`**
  (`ConfigLine.h:112-115`). Every reverse-seek re-anchor and reverse pivot is CI-green and
  physically impossible. Pre-existing hazard, independent of FHT, must be reconciled on the bench
  before any backwards-seeking recovery is written.
- **There is no mission for FHT to protect.** `MissionScript.cpp` is 16 lines with a single
  `.task("testOne")`; the tree is mid-tune with uncommitted config edits. Building fault recovery on
  an unauthored mission and an unfrozen tuning branch, to flash tomorrow, inverts the priority order.
- **Widening `OFF_MAP_GATE` is not a shortcut.** Flipping `ConfigNav.h:395` to `BEST_EFFORT` is one
  line and makes `RETURN_TO_NODE` accept an unverified dead-reckoned pose and keep driving — the
  single most likely way to convert a harmless wedge into a run that scatters placed tiles.

**Solvable challenges**
- **Mission step watchdog — genuinely hours of work.** `_stepStart_ms` is already set at
  `Mission.cpp:423` and never read. A `now - _stepStart_ms > cap` check → stop + park is ~10 lines,
  host-testable in the existing ctest harness. Highest-value tier-0 item, net-safe because its only
  action is to stop earlier than 120 s.
- **`Drive::_updateProfile` elapsed ceiling** — confirmed no ceiling and no no-progress check, so
  any async `driveDist().tShape()` can spin forever with motors energized. A generous (2–3×
  expected) deadline latching a new `Fault` is small and self-contained; the pattern already exists
  in `ManipBelt.h:415-425`.
- **Preserving the mission queue on fault.** `_enterFaultBeeping` destroys it; preserving it and
  advancing `_idx` is small, and the safe case already exists (`Mission.cpp:573-585`).
- **Task-level (non-nav) skip is the safe tier-0 win.** Promoting the manipulator's forced-"done"
  into a `hasFault()` a task can return `false` on gives real skip-and-continue with pose intact.
  Note `Tasks::abort()`'s sticky `_aborted` is only polled inside idle-wait loops (`Tasks.cpp:216`),
  never by Drive or Manipulator — it cannot currently interrupt a moving task.
- **Run clock.** None exists. A single `_runStart_ms` + "past T, abandon remaining steps and park"
  is trivial and is the prerequisite for any triage-style FHT.
- **Full FHT (later horizon) is architecturally supported.** `RETURN_TO_NODE` +
  `ReturnToNodePolicy` and `Localization.reanchorAtWall()` are the right primitives; ADR-009 already
  scopes the deferred lost-line sweep. What they need is physical: rear board wired, ≥1 authored
  reset node, measured Phase 3 numbers, a run clock.

**Evidence and reasoning:** Resources are not the constraint — the constraints are physical and
temporal. The rules confirm the premise (10.7.2) but weaken the payoff: 10.8 already preserves
banked points when the robot wedges, so "safe-park to preserve points" is largely what
`FAULT_AWAIT_RESET` already delivers, and tier-0's real gain is remaining points plus the time
tiebreak. "Recover and continue" is blocked in tier-0 because the only verified re-anchor paths are
unreachable (`N_RESET_NODES == 0`) or physically ungated (rear board). What *is* achievable in hours
and strictly net-safe is the bounding work: read `_stepStart_ms`, add a `_updateProfile` ceiling,
add a run clock, stop destroying the queue — each of which only ever makes the robot stop sooner.

### Thinker (branches)

1. **Auto-ack that skips the faulting step** — replace the human BTN_DOWN in `FAULT_AWAIT_RESET`
   with a `FAULT_AUTO_SKIP` state: stop wiping `_count`/`_idx`, advance `_idx` past the failed step,
   `PathTracker.clearFault()`, `Localization.declarePosition(lastKnownNode)`, resume — behind a
   `Config::FHT_AUTO_SKIP` flag so bench runs keep today's behavior. ~30 lines, one file, existing
   surfaces only.
2. **Author the reset nodes that already exist** — the wall-zero path is wired and CI-tested but
   dead because `N_RESET_NODES == 0`. Author 2–4 wall-adjacent reset nodes in `field_data.json` and
   emit a `NEAREST_RESET[N]` table offline so "get to the nearest reset point" is a table lookup,
   not a search. The feature the issue asks for is a *data* gap, not a code gap.
3. **Recovery-first mission DSL: on-fail branches** — make the mission a labelled graph:
   `goTo("A").onFail(SKIP_TO("label_b"))`, `task("place").retry(1).thenSkip()`,
   `checkpoint("mosaic_done")`. Recovery becomes something the operator *authors* per step, in the
   layer that knows what each step is worth.
4. **Mission checkpoints with rollback** — database transaction semantics: each step declares a
   `required_pose` node, the mission commits a checkpoint at each anchored arrival, any fault rolls
   back to the last checkpoint, re-localizes, replays forward. Faults cost "work since the last
   checkpoint" instead of "the rest of the run".
5. **Degraded-mode ladder instead of fault/no-fault** — named capability levels FULL (line + gyro +
   vision) → NO_LINE (blind hops on gyro + encoders) → NO_GYRO → CRAWL → PARK; every fault *demotes
   a level* rather than halting, and `PathTracker` replans over only the primitives that level
   supports. The aviation/spacecraft framing; ADR-041's blind-decay penalty is a natural hook for
   per-level edge-cost multipliers.
6. **Supervision tree over three fault stores** — replace `Drive::Fault` / `PathTracker::Fault` /
   Mission FSM with one OTP-style supervisor where each module declares a restart strategy
   (`restart_self`, `restart_subtree`, `escalate`) and a restart budget (N restarts in T seconds
   before escalating). Makes "restart the line follower and retry the hop" first-class and bounded.
7. **Minimal-risk manoeuvre as a designed behavior** — self-driving cars don't stop where they fail,
   they execute a pre-computed MRM: `parkSafely()` brakes, retracts the manipulator to a
   footprint-minimal pose, backs off any tile it may be resting on, kills the belt, holds — chosen
   per region so parking never destroys placed tiles. Independently useful as the terminal action
   for every other branch.
8. **The run clock is the real supervisor** — build the missing 2-minute clock first and derive
   every deadline from it: each step gets a budget from remaining time, overrun *is* the fault
   signal, and the terminal behavior at T-20 s is "return/park", not "await reset". Collapses the
   four wedge paths into one global preemption; converts FHT from an exception mechanism into a
   scheduling one.
9. **Fault earlier and more often, on pose confidence** — today the robot faults *too late* (50 cm
   off-map, motors energized, no ceiling on any blind hop). Compute a continuous pose-confidence
   scalar from encoder-vs-authored-edge residual, gyro-vs-authored-bearing residual, and
   hops-since-last-anchor; demote or re-anchor the moment it drops. Recovery from 5 cm is cheap;
   from 50 cm it is the heroics ADR-009 forbids.
10. **Analytical redundancy: diagnose which sensor lied** — four partially-redundant estimates of
    the same motion (L encoder, R encoder, gyro heading, line error + expected edge geometry). A
    residual/voting monitor says not just *that* something is wrong but *which channel* is wrong,
    then drops it. Classical FDI, no new hardware; turns LINE_LOST from a verdict into "the line
    board is unreliable, switch this leg to a gyro blind hop".
11. **Kidnapped-robot relocalization by bounded search** — when the offset cap blows, run a bounded
    "where am I": spiral/box search for any line, read `getJunctionType()`, intersect the
    junction-type signature with gyro heading against the ~20-node graph to get a hypothesis set,
    drive one edge to disambiguate. A discrete particle filter over 20 nodes with a 4-value
    observation fits in flash; the only branch that recovers a genuinely lost pose.
12. **Fault injection in the sim and CTPS** — you can't validate a recovery layer on hardware in a
    day, but you can in Python: add an injection layer to `sim/` and the parity harness that kills
    the line at hop *k*, freezes the gyro, or teleports pose by X cm, then asserts the policy's
    outcome (points preserved, in bounds, run completed) and renders it in CTPS. FHT becomes a
    CI-enforced property with a per-mission "survivability score".
13. **Reframe: the problem is observability, not recovery** — at a regional you get a handful of
    attempts, and today a fault yields three OLED letters and a dead run, so you can neither recover
    it *nor* learn from it. Invest in post-run forensics: dump the `EventTrace` ring plus a compact
    per-hop pose/residual log over Serial at fault entry, replay through CTPS, fix root cause
    between attempts. Reducing fault *frequency* may buy more points than recovering from faults.
14. **Reframe: make faults cheap by mission design** — if a fault at step 12 costs the whole run,
    the bug is the mission's structure. Re-author so scoring actions are independent, ordered
    highest-EV-first, separated by anchor points, so a fault only forfeits what hasn't been banked.
    Planner-side "fault-cost curve": for each step, how many points are still at risk if we die here.
15. **Pre-flight built-in test and go/no-go** — a large share of "faulted at comp" is bad state at
    *arm* time: gyro not zeroed, a line board offline (`isOnline()` exists and is unused as a gate),
    a servo not homed, battery sagging, `N_RESET_NODES == 0`, stale calibration. A 3-second pre-arm
    BIT that refuses to start with a specific diagnostic converts a mid-run mystery into a 10-second
    fix on the table.
16. **Actuator FHT: end the silent-success lie (speculative)** — `ManipBelt` timeouts force `done`,
    so Mission continues believing a tile was placed that never was. Give the manipulator a real
    fault surface, a bounded unjam routine (reverse belt, re-open grabber, one retry), and a
    "release cargo safely and yield the task" abort. The failure class most likely to produce
    *undetected* score loss rather than a visible stop.
17. **Independent hardware watchdog as the floor (speculative)** — every software recovery assumes
    `loop()` is still running. The RA4M1 hardware WDT (plus an STM32-side motor timeout) guarantees
    a wedged blocking site or brownout ends with motors de-energized rather than a robot driving
    into the mat at full PWM. Recovers nothing; bounds the worst case.

## Live direction (FINAL — narrowed by Diego after round 2)

> "I think for now we don't need to like invent more fault. We just need a way to handle every
> fault we already have. We can do the others when we are qualified into national."

**Scope: give each of the 17 EXISTING faults a defined non-human response.** No new detection, no
new watchdogs, no pose work, no new generated tables. Split on "had the robot moved when it fired?":
Class A (6 dispatch-time `PathTracker` faults, pose intact, robot stationary on a known node) →
SKIP_STEP; Class B (in-motion, pose suspect, plus all 5 `Drive` faults which surface as
`DRIVE_FAULT`) → PARK. Needs three changes: stop wiping the queue at `Mission.cpp:321-322`, a real
terminal PARK state that needs no human, and a header-only `FaultResponsePolicy.h`.

Written up in
[`2026-07-20-issue40-fht-fault-handling-design.md`](./2026-07-20-issue40-fht-fault-handling-design.md).

Everything below this line was the pre-narrowing direction, retained for the national horizon.

### Superseded direction — Branch #8 (run clock) + Branch #5 (degraded-mode ladder), composed

Forks resolved before the narrowing: **Fork A → two axes, sampled only at node arrivals** (#3);
**Fork C → freeze by default, offline safe corridor** (#10). Both deferred to national.

Diego picked both. They are the two axes of the same supervisor:
- **#8 is the TIME axis** — how much run is left, and therefore what a step is allowed to spend.
  Overrun *is* the fault signal; the terminal behavior is park, not await-reset.
- **#5 is the CAPABILITY axis** — what the robot can still do; a fault demotes a level
  (FULL → NO_LINE → NO_GYRO → CRAWL → PARK) instead of halting, and `PathTracker` replans over
  only the primitives that level supports.

Constraint carried forward from the Critic that any composed design must answer: **motion after a
fault has negative EV under the rules** (28-pt barrier bonus + placed tiles are scored on final
position), so demotion must be gated on expected points, not just on capability. And the pose
problem is real: `Localization` discards in-hop motion at the gate, so a demoted mode has no
in-hop displacement estimate to work from.

Target horizon: national `2026-08-24` / worlds `2026-11-30` for the ladder; the net-safe subset of
#8 (step deadline, `_updateProfile` ceiling, run clock, stop wiping the queue) is the only part
that is hours-scale.

## Perspectives (round 2 — Thinker only, focused on the #8 + #5 composition)

**Branches:**

1. **Points ledger as the demotion gate** — supervisor state is a two-number ledger:
   `banked_points` and `points_at_risk` (banked points within a wander radius of here). The offline
   pipeline emits `Routes::EDGE_RISK[N][N]` (`uint8_t` points-at-risk per edge, computed in
   `graph_builder.py` from barrier/target geometry already in `field_data.json`), so at fault time
   the firmware answers "moving from here endangers 28 barrier points" by table lookup. Demotion
   fires only when `expected_marginal_points > risk × P(wander)` — making the rules constraint a
   structural property of a generated table, not a comment in a policy header.
2. **The clock subsumes the ladder entirely (no capability enum)** — exactly one new concept: a
   `RunClock` armed at BTN_UP and a per-step deadline from remaining time. No FULL/NO_LINE/NO_GYRO
   enum at all; "degraded" is emergent because an overrunning step is abandoned and the next runs
   with a bigger share of the clock. All four wedge sites get the *same* check against the *same*
   clock. Buys honest thresholds (per constraint 8 you can't calibrate a physics-based stall
   threshold, but you *can* defend "this step got 9 s of a 120 s run"); costs ~90 s of discovery
   latency when a line is lost early.
3. **Node-arrival-only supervision (dodges the pose gate by construction)** — clock and capability
   are orthogonal, but the supervisor is evaluated ONLY at node arrivals, the exact instants where
   the nav gate is open and pose is known. Mid-hop the only permitted action is emergency
   stop-in-place — no demotion, no replan, no "estimate where I died". Concretely a
   `Supervisor::onArrival(node, health, now)` hook off PathTracker's SETTLE exit plus a dumb ceiling
   on every in-flight primitive. The ladder becomes a discrete-time system sampled at ~15 events per
   run — trivially enumerable in sim and CTPS.
4. **Offline-generated NEXT_HOP planes, one per capability level** — demotion is decided at runtime
   but replanning is not: `planner.py` emits `NEXT_HOP[L][N][N]`, one plane per level, each built by
   deleting the edge types that level can't execute. Firmware demotion is a single plane-index
   change in `FieldMap::getNextHop`, preserving "pathfinding lives off-board" and ADR-041's static
   philosophy. Payoff is a build-time validator assertion — "every scoring node is still reachable
   at NO_LINE, or the build fails" — provable in CI today, versus a runtime replan you can only
   hope works.
5. **Per-edge capability mask; no global ladder** — invert ownership: level is a property of the
   *world*, not the robot. `EdgeInfo` (`routes.h:52-61`) gains `uint8_t requires_mask`
   (LINE|GYRO|VISION|ENCODER); the supervisor holds only a `health` bitfield; `PathTracker` refuses
   a dispatch when `edge.requires & ~health` is nonzero — and refuses *at the node*, where pose is
   known, rather than discovering mid-hop. No ordering among degradations, so "lost vision, kept
   line" is expressible where a global ladder can't. Cost is known: `EdgeInfo` additions are a
   4-site edit (emitter, routes.h, sim, CTPS).
6. **Ship only the terminal behavior; ladder behind a compile flag** — the net-safe subset is
   deliberately NOT the supervisor: run clock + the four missing deadlines, with the terminal action
   changed from "5 beeps then FAULT_AWAIT_RESET" to PARK (brake, motors de-energized, LEDs red, no
   motion, no human touch — which also avoids General Rules 10.7.2). Every demotion path lands
   behind `#ifndef SUPERVISOR_ALLOW_DEMOTE / false`, matching the existing `MISSION_ALLOW_REARM`
   override idiom. Strictly "stops sooner and more safely" — nothing in it can move the robot
   anywhere new.
7. **Confidence as a decaying scalar, not a discrete ladder** — replace levels with one
   `float confidence ∈ [0,1]` that decays per blind-centimetre and restores to 1.0 at every
   line/reset anchor — the runtime mirror of ADR-041's offline `(1 + k·(d/d_ref)^exp)` decay, reusing
   an already-validated CI-tested model. Levels become bands; hysteresis is one threshold pair.
   Answers the pose constraint honestly: you can never know *where* you are mid-hop, but distrust is
   monotone in blind distance travelled, and that quantity you genuinely have.
8. **Budgets authored in the Mission DSL, not computed by a module** — no new module, no new fault
   store: the supervisor is the `Mission` engine enforcing operator-typed metadata,
   `Mission.goTo("A").budget(8000).worth(14).onOver(Over::SKIP)`. This is ADR-008's greedy scheduler
   minus the greed — ADR-008 already assumes every task carries an estimated duration and expected
   score, so the operator supplies them instead of a heuristic consuming them. Cheap here precisely
   because `MissionScript.cpp` holds one `.task("testOne")`: mission and budgets get written in the
   same sitting, by the person who knows both.
9. **A pure decision function in a policy header** — keep all three latched fault stores and add one
   header-only `DegradePolicy.h`:
   `Action decide(DriveFault, NavFault, remaining_ms, points_at_risk, health)` →
   `{CONTINUE, RETRY_ONCE, DEMOTE, PARK, FREEZE}`. Zero Arduino surface, so it links straight into
   the existing ctest harness alongside `TurnSignPolicy.h`/`StopAtLinePolicy.h` (ADR-030/049) and
   mirrors into `sim/runtime.py` for parity — the direct answer to "how do you validate this without
   hardware".
10. **Freeze by default; motion only inside an offline "safe corridor"** — not a ladder, a geofence.
    Default action on any fault is FREEZE (banking every point per 10.8). Post-fault motion is
    permitted only when the last known node is in an offline-marked safe set — nodes/edges with no
    barrier and no placed cargo within R cm, computed by the planner from `field_data.json` and
    emitted as a bitmask — and even then the only permitted move is a straight reverse back down the
    edge just traversed. Reframes the seed's "get to the nearest reset point" as "there are exactly
    three places on this field where moving while confused beats freezing, and here they are".
11. **The clock budgets points-per-second, not seconds** — track realized score-rate against the
    plan's. A step is abandoned not when it took too long but when its *marginal* points-per-second
    falls below the best remaining alternative's — ADR-008's greedy ordering heuristic reused as a
    watchdog instead of as a scheduler, needing no metadata beyond what ADR-008 already mandates.
    Overrun becomes contextual: 20 s on a 40-point cement delivery is fine, 20 s on a 7-point hop is
    a fault, and near end-of-run the return step's rate rises until it wins automatically — ADR-008's
    `t=100 s` guard falls out of the arithmetic instead of being hardcoded.
12. **Demotion by queue splice into an operator-authored salvage script** — fix
    `_enterFaultBeeping()`'s `_count = 0; _idx = 0` not by snapshotting but by *splicing*: on fault
    the remaining queue is replaced with a second hand-authored mission, `buildSalvageMission()`,
    next to `buildMission()` in `MissionScript.cpp`. No capability enum, no runtime replanning, no
    new tables — two scripts and one switch. Cheapest thing that still satisfies "skip the faulting
    task and keep competing", and both scripts render in CTPS today with no new tooling.
13. **Explicit health probes at known poses, never mid-hop inference (speculative)** — health bits
    are set only by deliberate self-test steps the mission spends time on (stop at a line node,
    confirm the array sees a line, compare yaw drift against the edge's `heading_deg`, check L/R
    encoder agreement), never inferred from a fault mid-move. Direct concession to constraint 8: a
    jam, a sagging battery and an I2C hiccup are indistinguishable *while moving* but trivially
    distinguishable at a standstill on a known line. Costs real clock time (~1 s per probe), making
    probe placement a scheduling decision the run clock has to price.
14. **Author reset nodes first; the ladder has exactly one demoted behavior (speculative)** —
    `N_RESET_NODES == 0` means the fully-wired CI-tested wall-zero interlude is dead code on the
    production field. This says the supervisor work is really field-data work: author 2–4 wall-zero
    reset nodes, and define the *entire* degraded mode as "reach the nearest reset node using only
    edges you can still execute, re-anchor, resume". One behavior instead of a four-rung ladder,
    built on an existing tested mechanism, and the literal implementation of the seed's third clause.
15. **Open the Localization gate during hops with an uncertainty envelope (speculative)** — attack
    the pose constraint head-on: let `Localization::update()` integrate during a hop and tag it with
    a growing radius, so at fault time you have `(offset, ±σ)` instead of a stale pre-hop reading.
    Every other branch works *around* the closed gate; this removes the constraint and unlocks the
    seed's "estimate position to get to the nearest reset point". Highest-risk shape on the list — it
    touches ADR-005 single-writer semantics, ADR-042's offset model, and the firmware↔sim parity
    harness, and frame-rebind/offset-staleness is already a known recurring bug source in exactly
    this code.
16. **A deadline registry below Mission (a heartbeat contract)** — no points, no levels: a
    structural fix to "four separate hand-written watchdogs". A `Supervisor` module at the `util`
    layer exposes `arm(Source, budget_ms)` / `disarm(Source)`; every module starting something
    time-consuming registers, and the supervisor ticks at ~124 Hz escalating anything past budget.
    The four wedge sites become four `arm()` calls — and `Manipulator`/`ManipBelt`, which today have
    no fault concept at all, gain one for free by registering. Clock and ladder then sit *above*
    this as policy, with the registry as the single mechanism both use.

## The three real forks (synthesis of round 2)

- **Fork A — does the clock subsume the ladder?** #2 (yes: one concept, emergent degradation,
  honest thresholds, slow discovery) vs #5/#7 (no: a capability can be lost with time to spare).
- **Fork B — where does "level" live?** Global enum · per-edge `requires_mask` (#5) · continuous
  confidence scalar (#7) · offline `NEXT_HOP[L][N][N]` planes (#4).
- **Fork C — what gates a demotion, given motion has negative EV?** Generated `EDGE_RISK` points
  ledger (#1) · marginal points-per-second (#11) · freeze-by-default geofence (#10).

Mechanism candidates that are orthogonal to all three forks and compose with any answer:
**#16** (deadline registry — the single mechanism both axes use, and it gives `Manipulator` a fault
concept for free), **#9** (`DegradePolicy.h` pure decision function — host-testable + sim parity,
the answer to "validate without hardware"), **#6** (net-safe shippable subset with demotion behind
a compile flag).

## Killed / parked

- Branch #15 (pre-flight BIT) — not selected this round, but cheap and independently valuable.
- Branch #2 + #7 (author reset nodes + designed safe-park) — not selected; note the ladder's PARK
  rung and #8's terminal behavior both need #7's `parkSafely()` eventually.
- Branch #13 (observability reframe) — parked; still the cheapest lever between regional attempts.
- Seed mechanism "estimate position to reach the nearest reset point" — **killed as scoped**: no
  in-hop pose estimate exists (`Localization.cpp:359-390` gate discard) and no reset node exists
  (`routes.h:123`).
- Seed mechanism "try to find line" — **parked as dangerous**: re-runs a deterministic failure, and
  its success case (latching a *different* edge's line) is worse than its failure case.

## Decisions & debate verdicts
