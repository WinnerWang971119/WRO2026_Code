# WRO 2026 — Design Decisions Index

This file is an **index** into the architecture decision records (ADRs) under [`docs/decisions/`](./docs/decisions/). Each ADR captures a single decision in the form: **rule in force**, **why**, **how to apply**, **trigger to revisit**, **status**. The original historical prose has been moved out; commit messages and PR descriptions retain the implementation play-by-play.

## When to write a new ADR

- A new design rule that crosses module boundaries.
- A change to an existing rule (supersede the old ADR; do not edit it in place except for status).
- A "rule" that lasted long enough to be worth citing (a one-PR convention isn't worth an ADR; a project-wide constraint is).

## Index

### Foundational architecture

- [ADR-001 — Lines + intersections, no wall-following, no dead-reckoning](./docs/decisions/001-no-wall-follow-no-dead-reckon.md)
- [ADR-002 — Three-layer mental model (perception → map → planning)](./docs/decisions/002-three-layer-mental-model.md)
- [ADR-003 — Arduino-style modules — no class hierarchies, no DI, no `delay()` in `update()`](./docs/decisions/003-arduino-style-modules.md)
- [ADR-004 — Motor-speed/position PID stays on the STM32](./docs/decisions/004-pid-stays-on-stm32.md)
- [ADR-005 — `Localization` is the single source of truth for current node + heading](./docs/decisions/005-localization-single-source-of-truth.md)
- [ADR-022 — field_data geometry lives in JSON; field_data.py is a loader](./docs/decisions/022-field-data-json-loader.md)

### Mission-layer

- [ADR-006 — Read the mosaic with a camera at a line-anchored stop](./docs/decisions/006-mosaic-via-camera.md) (Phase 3)
- [ADR-007 — Manipulator exposes control primitives only until the mechanism is built](./docs/decisions/007-manipulator-control-primitives-only.md) (Phase 3)
- [ADR-008 — Greedy score-per-second scheduling + 2-minute time guard](./docs/decisions/008-mission-scheduling-and-time-guard.md) (Phase 4)
- [ADR-009 — Three named failure modes get explicit bounded recovery](./docs/decisions/009-failure-recovery-first-class.md) (Phase 4)
- [ADR-010 — Lightweight in-memory logging only — `EventTrace` + Serial heartbeat](./docs/decisions/010-lightweight-logging-only.md)

### Drive-layer

- [ADR-011 — Static motor-bias scalar (Strategy B); C/D on the shelf](./docs/decisions/011-motor-bias-strategy-b.md)
- [ADR-012 — Fluent builder for any verb with 3+ knobs](./docs/decisions/012-fluent-builder-3plus-knobs.md)
- [ADR-013 — `LineFollower.setEnabled(false)` does not implicitly brake](./docs/decisions/013-linefollower-disable-no-implicit-brake.md)
- [ADR-014 — T-shape trapezoidal velocity profile, acceleration-based, opt-in](./docs/decisions/014-tshape-trapezoidal-velocity-profile.md)
- [ADR-015 — Drive owns a FIFO command queue; destructors enqueue](./docs/decisions/015-drive-command-queue.md)
- [ADR-016 — Drive builders block by default; `.async()` opts out](./docs/decisions/016-sync-default-builders.md)
- [ADR-017 — `DriveMode::SYNC` + `tShape` are the project-wide default](./docs/decisions/017-sync-plus-tshape-default-profile.md)
- [ADR-018 — LineFollower distance-gated stops use symmetric trapezoidal + active brake](./docs/decisions/018-linefollower-distance-gated-stops.md) (Refined by ADR-023)
- [ADR-022 — Dual motor profile via per-motor structs + ActiveMotor alias](./docs/decisions/022-dual-motor-profile.md)
- [ADR-023 — Pass-through collinear intersections + runtime overshoot-mode probe](./docs/decisions/023-pass-through-collinear-intersections.md)

### Perception

- [ADR-019 — Custom 8-channel `WROLineArray` replaces vendor `MatrixLineTracer`](./docs/decisions/019-8channel-custom-line-sensor.md)
- [ADR-020 — Freeze LineFollower PID output during any intersection](./docs/decisions/020-junction-pid-freeze.md)
- [ADR-021 — Camera is the sole color sensor — no dedicated `ColorReader` module](./docs/decisions/021-camera-as-sole-color-sensor.md) (Phase 4)

## Status summary

| Status | Count | ADRs |
|---|---|---|
| Implemented | 18 | 001, 002, 003, 004, 005, 010, 011, 012, 013, 014, 015, 016, 017, 018 (refined by 023), 019, 020, 022, 023 |
| Partially implemented | 2 | 008 (Mission DSL shipped; greedy scheduler + time guard deferred to Phase 6), 009 (lost-line + missed-intersection in Phase 1/2; failed-grip pending manipulator) |
| Pending implementation | 3 | 006 (Phase 4 camera selection), 007 (Phase 5 manipulator), 021 (Phase 4 camera-as-sole-color-sensor) |
| Deferred | 0 | — |
| Superseded | 0 | (ADR-010 supersedes the older "no logging at all" stance from the Phase 0 draft) |

## Conventions

- ADR numbers are assigned **in order of first appearance**, not by topic. Never renumber.
- When a decision changes, write a new ADR with the next number, mark the old one **Superseded by ADR-NNN**, and update both the "Supersedes" / "Superseded by" cross-links.
- Each ADR stays one self-contained page (~30–60 lines). If a decision genuinely needs more, link out to a design doc under `docs/`; do not let the ADR grow.
