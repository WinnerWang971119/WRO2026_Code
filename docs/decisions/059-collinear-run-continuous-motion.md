# ADR-059: Collinear runs chain velocity across hops instead of stopping at every node

**Status:** Implemented
**Date decided:** 2026-07 (phase)

## Rule

A collinear run of hops (LINE, PROJECTION, or BLIND) drives as one continuous
motion — a single ramp-up at the run's start, cruise held through every
intermediate node, a single ramp-down/brake at the run's end — instead of
braking and re-accelerating at each node. Run-scoping and chain-speed
resolution live entirely in `PathTracker`'s walk-ahead plus a pure, header-only
`ChainSpeedPolicy` (a **two-pass** junction planner: forward accel-limit,
backward decel-to-floor, clamped to `[pwmMin, |cruise|]`); `LineFollower` and
`Drive` gain **no new cross-hop controller state** — the terminal, per-hop
scope ADR-050 relies on stays terminal. Ramp-down targets the floor
(`(cruise − PWM_MIN)/accel`, shed-to-floor), not zero; the run's final hop is
always an ordinary `cruise→floor→brake` stop, and the blind-chain final hop's
dispatched tShape end speed is pinned to the floor (`±PWM_MIN_MAGNITUDE`),
not 0 — a cruise→0 shed (17 cm at 85/5) cannot fit a short final hop and
triangle-collapses the profile, launching the hop below the cruise the
chassis physically enters at; the cruise→floor shed (7 cm) fits, the launch
holds the cruise entry, and the brake completes floor→rest. `entryPwm`
preserves the
ADR-023 cruise carry: a chain opening right after a pass-through seeds the
captured cruise, never the cold-start floor. Inter-hop boundary carry (the
~1.5–3 cm the chassis travels holding the last PWM across a node boundary) is
measured via encoder delta and subtracted from the next hop's commanded
distance, never assumed zero.

`PROJECTION_FOLLOW` now chains `.setTShapeRampDown(!projPassThrough)`, so a
`line–proj–line` pass-through actually holds cruise across the projection
node instead of shedding to floor and re-accelerating on every hop. Blind
chains are new firmware behavior, default ON behind
`PATH_BLIND_CHAIN_ENABLED`, `PATH_BLIND_CHAIN_MAX_CM`, and `MAX_CHAIN_HOPS`.
The CTPS run-grouper renders a whole collinear run (line, projection, and
blind) as ONE velocity profile; a run is bounded by braked stops, and
zero-motion records (`ACT_HEADING_RESET`, `declare_position`, manip/wait) are
transparent to it. The single `Config::NAV_TURN_TOLERANCE_DEG` (0.5°) knob is
unchanged — no separate pass-through tolerance, no skipped-turn split.

Two wire surfaces carry chain data and are kept distinct: the CTPS
command-stream reuses the existing optional `start_speed`/`end_speed` fields
(no new field); the ADR-050 strict parity record's drive/blind kind gains
`start_pwm`, `end_pwm`, and `chain` (both firmware and sim emit them —
`parity.py` raises on an unknown *or missing* field of a known kind, so the
bump lands on both sides together).

## Why

ADR-058's phase-object kernel made the firmware's real per-hop stop shape
visible in CTPS, which surfaced two problems this ADR fixes together: LINE
pass-through chains already produce a whole-run trapezoid in firmware
(ADR-023/ADR-046), but CTPS was rendering it as a lie (a stop keyframe at
every intermediate node); and PROJECTION pass-through chains do not hold
cruise in firmware at all — `PROJECTION_FOLLOW` armed `setBrakeOnGate(false)`
but never `setTShapeRampDown(false)`, so each projection hop sheds 85→50 and
snaps back, a real motion defect a truthful CTPS render would otherwise ship
as a *second* lie. BLIND chains have no line to fall back on and no
mid-chain re-anchor, so the fix there is new firmware: every stop→launch is a
static-friction break-away that spikes heat and slips the wheels, corrupting
the next move's heading (memory: `project-fewer-stops-better-friction-drift`)
— a continuous dead-reckoned run measurably drifts less than N stop-starts
through the same blind nodes.

The two-pass (forward+backward) planner is required, not a convenience: a
decel-only formula (`min(cruise, pwmMin + accel·distToEnd)`) passes unit
goldens but produces a half-clamp slam on a short final hop — observed at
17 cm of naive ramp room clamped into 5 cm, a 3.4× over-accel spike the
forward pass rules out by construction. Run-coalescing into one multi-hop
Drive command was considered and rejected: it would remove the boundary-carry
and short-hop defects chaining must otherwise handle, but it breaks the
hop-granular ADR-050 parity trace and needs a new mid-move distance observer;
the decision stands on parity granularity, not on those defects being
non-issues. A separate pass-through tolerance constant was also rejected —
pass-through is definitionally "the turn would have been skipped," so
splitting the two thresholds is incoherent; the fix for a field that doesn't
pass through cleanly is to straighten the field, not add a second knob.

## How to apply

- Chain-speed math lives only in `ChainSpeedPolicy.h` (firmware, header-only,
  pure, host-testable) and its Python mirror `chain_speed.py`, golden-pinned
  byte-identical (float32/int16 discipline, matching `profile.py`). Never
  duplicate the two-pass math inline in `PathTracker` or the sim.
- `PathTracker`'s walk-ahead is the only place a run is assembled: guard
  chain-eligibility at pivot/seek/reset/goal nodes, direction flips,
  non-collinear bends, and hops below `TSHAPE_MIN_MOVE_CM` — end the chain
  before the unsafe hop, which then takes the ordinary braked path.
  `LineFollower`/`Drive` dispatch chained mid-hops via the existing builder
  surface (`setStartSpeed`/`setEndSpeed`/`setBrake(false)`); they hold no run
  memory of their own. The run's FINAL hop dispatches the braked statement
  with `setEndSpeed(±PWM_MIN_MAGNITUDE)` — the floor, never the braked
  default 0 — so a short final hop's cruise→floor shed fits (no
  triangle-collapse, no re-dip below the run's cruise entry) and the brake
  takes floor→rest. This is a dispatch detail only: the ADR-050 parity record
  still carries the PLAN endpoint (`end_pwm = 0`) on every braked hop.
- The CTPS run-grouper (`track_builder.py`) treats a run as bounded by braked
  stops and folds zero-motion records through transparently; it never
  re-derives a chain decision — it consumes the recorded/resolved speeds.
- Bump the ADR-050 parity schema (`start_pwm`/`end_pwm`/`chain`) on the
  firmware emitter and the sim together, in the same change — the strict
  schema faults loudly on either side landing alone.
- Guard new blind chaining behind `Config::PATH_BLIND_CHAIN_ENABLED` (default
  true), `PATH_BLIND_CHAIN_MAX_CM`, and `MAX_CHAIN_HOPS`; disabling the first
  restores today's stop-at-every-node behavior exactly. `PATH_BLIND_CHAIN_MAX_CM`
  is set high (a loose runaway guard, not a physical constraint — owner
  decision, 2026-07): `MAX_CHAIN_HOPS` is the practical bound on how far a
  chain runs. See "Trigger to revisit" below.

## Trigger to revisit

- If the chassis cannot brake to rest from the PWM floor due to inertia on a
  short final hop (bench), revisit the final-hop end speed (owner: Diego,
  2026-07).
- Bench data shows the measured inter-hop boundary carry is noisier than the
  ~1.5–3 cm model (loop-rate dependent) — the subtraction may need a
  measured-with-margin or filtered form instead of a raw encoder delta.
- A future field or mission genuinely needs a mid-run stop inside an
  otherwise-collinear chain (e.g. a manipulator pickup) — today's model has
  no "stop here anyway" annotation; see ADR-023's own trigger for the same
  gap on pass-through.
- Run-coalescing's rejected tradeoff shifts — e.g. a mid-move distance
  observer becomes cheap for another reason — reopen the coalescing decision
  against the parity-granularity cost recorded here.
- Bench testing shows a blind chain that ends directly into a
  `Drive.stopAtLine()` seek drifts past that seek's `STOP_AT_LINE_ARM_WINDOW_CM`
  arm window. `PATH_BLIND_CHAIN_MAX_CM` was deliberately decoupled from the
  arm-window budget and set high (owner decision, 2026-07) because
  `MAX_CHAIN_HOPS` is the intended practical bound; if hardware proves that
  insufficient, add a SEPARATE, tighter bound scoped to a chain-into-seek
  handoff rather than re-tying this general cap to the arm window.

## See also

- [ADR-023 — Pass-through collinear intersections](./023-pass-through-collinear-intersections.md) — amended: the `entryPwm` cruise-carry seed this ADR specifies is the same carry ADR-023 established at the LINE→BLIND handoff.
- [ADR-046 — Projection is a node property, not an edge type](./046-projection-as-node-property.md) — amended: the projection ramp-down fix closes the gap where `PROJECTION_FOLLOW` never chained `setTShapeRampDown`, so its distance-gated pass-through now actually holds cruise.
- [ADR-052 — Wall-free heading-only re-anchor at collinear line pass-throughs](./052-line-arrival-heading-reset.md) — amended: the pass-through arrival this ADR re-anchors at is unchanged in trigger, but now may sit inside a chained run rather than a single hop.
- [ADR-058 — CTPS playback renders the firmware's T-shape motion profile](./058-ctps-firmware-motion-profile.md) — amended: the phase-object kernel that made the per-node stop-and-restart lie visible; this ADR's run-grouper builds one profile over the whole run instead of one per hop.
- [ADR-050 — PathTracker sim↔firmware FSM parity](./050-pathtracker-sim-parity.md) — the strict trace schema whose drive/blind record kind gains `start_pwm`/`end_pwm`/`chain`; the terminal-classifier scope this ADR is careful not to widen.
- [ADR-018 — LineFollower distance-gated stops](./018-linefollower-distance-gated-stops.md) — the `cruise→floor→brake` stop shape every chain's final hop still uses unmodified.
- Memory: `project-fewer-stops-better-friction-drift` — the bench rationale for treating blind chains as high-priority, not merely in-scope.
- `docs/plans/2026-07-10-collinear-run-continuous-motion.md` — the originating plan.
