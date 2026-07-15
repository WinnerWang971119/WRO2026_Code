# ADR-032: Manipulator API — verb-level builders over two actuator classes

**Status:** Implemented (structure); calibration constants TBD at hardware bring-up
**Date decided:** 2026-06 (Phase 5 pulled forward)

## Rule

The `Manipulator` facade exposes coordinated semantic verbs (`grab`, `release`, `sortTo`, `dump`,
`reset`) as blocking-by-default builders with an `.async()` opt-out, per ADR-012.

Internally the facade owns two actuator classes:
- **`ManipServo`** (×3 instances: grabber, sorter, dumper) — fire-and-forget RC servo wrappers
  whose `isDone()` is time-based (no position feedback in the vendor RC library).
- **`ManipBelt`** (×1) — belt drive with real encoder feedback; `isDone()` is position-based for
  mid-station stops and time-based for end-station stops (driving into a hard end-stop stalls the
  encoder).

`home()` is stall-home: the belt runs toward the end-stop until the encoder stalls.

Semantic verbs may fire multiple actuators concurrently where the mechanism allows it (e.g. open
gripper while rotating sorter). Primitives (`setAngle`, `beltTo`) remain public for calibration
and unit tests; mission code should use verbs.

This supersedes ADR-007's "no high-level action verbs until the mechanism is built" stance. The
CAD is finalized; the API shape is locked. Verbs ship as thin scaffolds until the mechanism is
assembled and calibrated.

## Why

RC servos in the MatrixMiniR4 library expose no position feedback. A uniform blocking `isDone()`
that trusts servo position would be dishonest and would cause timing regressions when servo load
changes. Separating `ManipServo` (time-based) from `ManipBelt` (encoder-based) makes the
honesty boundary explicit and lets the facade compose them correctly.

A per-actuator blocking default would serialize motions that the mechanism can overlap (e.g.
sorting while the belt indexes). The verb-level builder keeps ADR-012's knob (`.async()`) at the
right granularity: the operator decides whether the verb as a whole blocks, not each actuator
within it.

## How to apply

- Mission code calls `Manipulator.grab().async()` / `Manipulator.sortTo(BIN_A)` etc.
- Calibration sessions call `Manipulator.setAngle(GRABBER, 90)` / `Manipulator.beltTo(MID_1)`.
- `ManipServo::isDone()` checks elapsed time against a `ConfigManipulator.h` timeout constant.
- `ManipBelt::isDone()` checks encoder delta; stall-home is a separate timed-stall detection loop.
- End-station belt moves use time-mode to avoid a false "done" on stall before reaching target.
- `Manipulator` is not in Drive's command queue — it owns its own completion state.

## Trigger to revisit

- A second robot with a different manipulator geometry needs a different verb set.
- The RC library gains position feedback, making `ManipServo::isDone()` encoder-based.

## See also

- [ADR-007](./007-manipulator-control-primitives-only.md) — superseded; original "primitives only" stance
- [ADR-012](./012-fluent-builder-3plus-knobs.md) — fluent builder rule; verbs with 3+ knobs use builders
- [ADR-016](./016-sync-default-builders.md) — blocking-by-default builder policy
- [ADR-022](./022-dual-motor-profile.md) — motor profile ADR; belt motor is a separate DC channel
