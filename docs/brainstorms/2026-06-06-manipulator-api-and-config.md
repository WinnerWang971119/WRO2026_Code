# Brainstorm: Manipulator API (#25) + Better Config (#39)

**Seed (verbatim):** Issue#25 Remember + Issue#39

**Context summary:**
- **Issue #25 — Manipulator:** CAD/blueprint now done → mechanism decided: 3 servos
  (Grabber, Sorter, Dumper) + 1 LEGO motor (belt slide). Wants a Drive-style API —
  blocking by default, opt-in async — so teammates code without mistakes. Current
  `Manipulator.h/.cpp` stub is stale (exposes `setLift/setGripper/setArm`, which don't
  match the new actuator set). ADR-007 parked it "control-primitives-only until the
  mechanism is built"; Roadmap slots Manipulator software in Phase 5. Belt motor is an
  encoder DC motor (`MiniR4.M1`/`M4` family) → position-bounded, more Drive-like than the
  servos.
- **Issue #39 — Config:** `Config.h` ~990 lines, one flat `namespace Config`. Messy at
  comp; can't adjust quickly. Wants split into global / driveTrain / manipulator + move
  data-check (`static_assert` validation) to its own file. **Tension:** mem0 records a
  standing "non-negotiable" preference for a SINGLE Config.h centralizing every constant.
  Config.h currently has ZERO manipulator constants — the new Manipulator block will
  worsen the exact mess #39 targets.
- **Coupling:** #39 explicitly names "a config for Manipulator" as one of the three split
  targets, and #25 will author that block. Designing the Manipulator API + its config is
  the natural moment to decide config organization.
- **Reference patterns:** Drive = fluent builders + FIFO queue + blocking-default/.async()
  (ADR-012/015/016); ADR-012 = fluent builder only for verbs with 3+ knobs, direct call
  for 0-2.

**Clarified framing:** Design the **Manipulator software API for Robot1** — 3 servos
(Grabber, Sorter, Dumper) + 1 LEGO belt DC motor — with Drive-style ergonomics (blocking
by default, opt-in `.async()`) so teammates author Mission/Task code without mistakes.
Jointly decide **how to organize Config**: single Config.h vs split (global / driveTrain /
manipulator) is an OPEN fork to debate — not pre-decided — including where the new
Manipulator config block lives and where the validation `static_assert`s go. Constraints:
Robot1 is the concrete target; a Robot2 with a different manipulator design *may* come
later (mirrors the existing two-chassis `ActiveMotor` swap pattern) but is out of scope
now — light extensibility only. Intent is to **implement now**, pulling Manipulator
forward from Roadmap Phase 5. Audience: the 2-3 person team writing mission code.

## Perspectives (round 1)

### Critic
1. **Drive-style blocking/async is unbuildable for the servos** — no motion-completion feedback. `MiniR4RC::setAngle` returns on I2C ACK; no position sensor / in-motion bit. A "blocking" `grabber.close()` can only `delay()` a guessed travel time (magic number, banned; physically wrong as load/voltage vary). Manufactures the exact silent-timing-corruption ADR-016 exists to kill, but here unfixable. Only the belt DC motor has a real done bit (`rotateFor`/`ChkRotateEnd`) — incompatible substrate with the servos.
2. **Pulling forward reverses ADR-007 on a false trigger** — CAD-done ≠ built/wired/calibrated. ADR-007's trigger is "mechanism v0 LANDS"; the stub warns "do not add hardware calls until at least built; premature abstraction will be ripped out." Servo open/close angles, detents, throw, belt travel are exactly the constants only measurable after assembly. Spends the implementation budget twice.
3. **Most verbs have 0-2 knobs → ADR-012 forbids the builders the proposal is built around.** A servo go-to-angle is the direct-call case. Forcing builders means re-implementing Drive's hardest machinery (RAII move-only builder + FIFO + `_drainBlocking` + tick re-entry) for trivial setpoint writes — CLAUDE.md rule 9's banned speculative abstraction.
4. **Blocking-by-default + cooperative tick = known re-entrancy footgun.** ADR-016 `@warning`: the registered tick callback must not issue sync-blocking commands or it recurses. Task bodies call subsystems directly from inside `Mission.update()` (the tick chain); a sync-blocking manipulator call from a task body is exactly that forbidden case. Two blocking spinners (Drive + Manipulator) can nest.
5. **Config split contradicts the standing non-negotiable single-Config.h rule AND breaks the static_assert safety net.** The `MOTOR_LEFT/RIGHT_ID` asserts sit under a 30-line comment naming the Drive.cpp lines they protect; relocating them to a `checks.h` strips the context, and there's no test proving a moved `static_assert` is still in a TU — a split can silently disable the net.
6. **Forces a premature Robot1/Robot2 swap abstraction; the `ActiveMotor` pattern it'd mirror is itself documented as leaky** (bias re-cal + M2/M3 hardcoding break the "one-line swap" promise). Designing Robot2's manipulator boundary before Robot1's is built = textbook premature generalization.

### Feasibility — VERDICT: **Feasible** (no hard blockers)
- **Belt:** genuine non-blocking completion via `MiniR4DC::rotateFor` + `ChkRotateEnd` / `getDegrees` target compare (`MiniR4DC.h:128/224/181`). Drive-style blocking-default/`.async()` works cleanly here.
- **Servos (the catch):** `MiniR4RC.h` exposes only `setAngle(0..180)`; `MMLower.h` has only `SetServoAngle`/`SetServoAngleRange` — **no servo position read anywhere in the vendor library.** Any servo `isDone()` MUST be timer-based (configured ms-per-degree / settle time), not feedback. Honest open-loop; document like Drive documents its single-task-done-bit constraint.
- **Don't reuse Drive's FIFO/done-flag** — it's single-slot, wheel-exclusive ("MUST NOT issue a bounded move and a turn concurrently"). Manipulator needs its own lightweight completion state but can copy the *ergonomic pattern* + the `setTickCallback(cooperativeTick)` spin. Already wired: `.ino` ticks `Manipulator.update()` (line 143) and calls `begin()` (line 196).
- **Config split:** compile-organization task, not a correctness risk. `constexpr` in a namespace = internal linkage, no ODR hazard across headers. Only real constraint is **include order** so each `static_assert` sees its symbols — keep each assert beside its symbols, or in a last-included `ConfigChecks.h`. Arduino IDE already compiles multiple project headers.
- **Implement-now:** practically feasible; only needs RC/M channel assignments fixed in Config (a data decision, not a code blocker).

### Thinker (branches)
1. **Asymmetric API — direct servos, fluent belt only.** Servos get plain `grabber(angle)`/`sorter(angle)`/`dumper(angle)` (1 knob, fire-and-forget); only the encoder belt gets `beltTo(deg).setSpeed().async()`. Honors ADR-012 literally.
2. **Semantic-verb facade over primitives.** `grab()`/`release()`/`sortTo(Bin)`/`dump()`/`beltRun(cm)`, each a tiny sequenced state machine ticking in `update()` with real `isDone()`; blocking/async lives on the *semantic* verbs. Raw primitives demoted to advanced/calibration. Highest "code without mistakes" altitude.
3. **Per-servo named sub-objects** mirroring `MiniR4.M1`/`RC1`. `Manipulator.grabber.open()`, `Manipulator.belt.run(20).async()`. Each sub-object is the natural home for that actuator's calibrated constants → couples API shape to config layout.
4. **Config split by owning-module** — constants live next to the module that owns them. `Config.h` keeps only global (battery, PWM bounds, presets); `DriveConfig.h` / `ManipulatorConfig.h` / `LineConfig.h` (or nested `Config::Drive` namespaces). Comp-day: open the one file for the subsystem you're tuning.
5. **Profile-struct + `ActiveManipulator` alias** mirroring ADR-022. `struct Robot1Manip { GRABBER_OPEN_DEG; ... BELT_PPR; GRABBER_RC_ID; }` + `using ActiveManipulator = Robot1Manip;`. Robot2 = second struct + one alias line. The "light extensibility" the framing asked for, in a pattern the team already accepts.
6. **Extract validation into `ConfigChecks.h` (the coupling branch).** All `static_assert`s — existing + new manipulator ones (servo IDs distinct & in `[1..4]`, belt ID ≠ MOTOR_LEFT/RIGHT_ID, open<close) — into one file that `#include`s every config header. Directly answers #39's "validation in another file" AND catches the new manipulator's biggest footgun (ID collisions) at build time.
7. **Manipulator as Tasks-registry bodies, not a verb DSL.** Keep `Manipulator` minimal (`setServo`, `belt`, `isDone`); semantic actions live as registered Task bodies invoked via `Mission.task("grab")`. Blocking falls out free (task bodies are blocking, ADR-025). Avoids duplicating Mission+Tasks machinery inside Manipulator.
8. **Single coordinated-pose builder (speculative).** `Manipulator.pose(Pose::CARRY)` resolves to a full {grabber,sorter,dumper,belt} setpoint tuple in config; blocks until all settle. Illegal intermediate states (gripper open while dumper tips) become unrepresentable. Assumes motions decompose into a fixed pose set — unknowable until built.

## Live direction
Converging on: a thin **coordinated semantic layer** (`grab`/`release`/`sortTo`/`dump`/
`reset`) that is **blocking-by-default** and fires its actuators **concurrently** inside,
sitting over per-actuator addressing. Servos = fire-and-forget primitives with
**timer-based** `isDone()` (no feedback — verified, see below); belt = encoder DC motor
with **real** feedback `isDone()` (`rotateFor`/`ChkRotateEnd`/`getDegrees`). Naive
per-actuator blocking-by-default is REJECTED — it serializes overlapping motions (e.g.
grab + sortTo together). `reset()` is a first-class coordinated/blocking verb (run-start +
failed-grip recovery + establishes belt zero).

**Settled (round 2):**
- Belt control model = **position / indexed** (`beltTo(pos)` absolute via `getDegrees`; needs
  homing zero established by `reset()`).
- Execution default = **async primitives + blocking semantic verbs, with `.async()` on the
  verbs** to opt out. Verbs fire their actuators concurrently inside and wait once.
- Addressing = **two actuator classes, four named instances**: one `ManipServo` class
  (instances `grabber`/`sorter`/`dumper`, fire-and-forget, timer `isDone()`) + one `ManipBelt`
  class (encoder feedback, `beltTo`/`home`, real `isDone()`). Mirrors `MiniR4RC<ID>` /
  `MiniR4DC<ID>` + CLAUDE.md rule 3. NOT four bespoke classes. Each actuator's `constexpr`
  config struct pairs 1:1 with its instance (= the Issue #39 manipulator config block).
- Coordinated verbs `grab`/`release`/`sortTo`/`dump`/`reset` on the facade return a tiny
  builder (for `.async()`) — consistent with ADR-012 (blocking/async is the knob, like
  `Drive.turnDeg`).
- **Primitives stay public.** Per-actuator direct calls remain usable: `grabber.setAngle(deg)`
  (servo, angle 0-180) / `belt.beltTo(deg)` (belt, encoder degrees), both fire-and-forget
  (never block; poll `isDone()` if needed). Convention: mission code uses the semantic verbs
  (concurrency-aware, mistake-proof); primitives are for calibration / bring-up / edge cases.

**Config organization — DECIDED by user (2026-06-07): SPLIT.** (User made the call directly,
overriding the /debate route.) Proposed structure:
- `Config.h` = thin umbrella that `#include`s the sub-headers in dependency order, last being
  the checks file. Everyone includes `Config.h` only (never a sub-header directly).
- `ConfigGlobal.h` — battery, `PWM_MAX`/`PWM_MIN_MAGNITUDE`, `SpeedPreset`, OLED/event,
  Mission/Tasks limits.
- `ConfigDriveTrain.h` — motor IDs + reverse, `MatrixTT`/`LegoEV3` + `ActiveMotor`, PID,
  geometry, turn, tShape, queue, stop-at-line, line PID.
- `ConfigManipulator.h` — NEW: per-actuator `constexpr` structs (grabber/sorter/dumper/belt:
  RC/M id, home/open/close/detent angles, ms-per-deg, belt PPR/speed/home) + optional
  `ActiveManipulator` alias for a future Robot2 (Thinker #5).
- `ConfigChecks.h` — ALL `static_assert`s (existing + new manipulator: servo IDs distinct &
  in `[1..4]`, belt M-id ≠ MOTOR_LEFT/RIGHT_ID, open<close), **included LAST** so every symbol
  is defined. Satisfies #39's "datacheck in another file."
- (Line-sensor config can become a 4th bucket `ConfigLine.h` or stay in Global — plan detail.)
- **Risk to manage** (Critic #5 + Feasibility): moved asserts must stay compiled — guaranteed
  by the umbrella always including `ConfigChecks.h` last; each assert keeps a comment pointer
  to the constant + code it guards (preserve the context the inline comment block had today).
- **Supersedes** the standing "single Config.h is non-negotiable" preference → flag for /save
  to reconcile the contradicting memory.

## Killed / parked
- Uniform Drive-style blocking facade across all 4 actuators — servos have no completion
  feedback, so a uniform blocking `isDone()` would be a lie (Critic #1, verified in headers).
- Reusing Drive's FIFO/done-flag queue — it's single-slot & wheel-exclusive (Feasibility).

## Decisions & debate verdicts
- **VERIFIED (source-cited):** `MiniR4RC.h` exposes only `begin`/`setHWDir`/`setAngle` — no
  servo position getter anywhere; `MMLower` has only servo *setters*. Servo `isDone()` must be
  timer-based. Belt (`MiniR4DC.h`) has `rotateFor`+`ChkRotateEnd`+`getDegrees` → real feedback.
