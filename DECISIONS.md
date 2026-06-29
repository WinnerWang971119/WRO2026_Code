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
- [ADR-005 — `Localization` is the single source of truth for current node + heading](./docs/decisions/005-localization-single-source-of-truth.md) (Amended by ADR-042, ADR-043)
- [ADR-042 — `Localization` tracks a 2D operator-offset delta from the last anchored node](./docs/decisions/042-operator-offset-localization.md)
- [ADR-043 — Wall-zero reset nodes: a `"reset"` node kind re-anchors pose against a known wall](./docs/decisions/043-wall-zero-reset-nodes.md) (Amends ADR-005)
- [ADR-041 — Blind-decay routing: convex multiplicative penalty on blind-edge distance to nearest anchor (line or reset)](./docs/decisions/041-blind-decay-routing.md)
- [ADR-022 — field_data geometry lives in JSON; field_data.py is a loader](./docs/decisions/022-field-data-json-loader.md) (Partially superseded by ADR-047)
- [ADR-047 — JSON-native EDGES overrides + field-sourced PIPELINE_CONFIG; `primitive_mode` string-tag encoding; geometry optional in override entries; `ast_rewrite.py` retired](./docs/decisions/047-json-edges-overrides-and-pipeline-config.md)
- [ADR-028 — FieldMap accessors bounds-check every index and return a safe sentinel](./docs/decisions/028-fieldmap-accessor-bounds.md)
- [ADR-029 — The validator is the single anti-ambiguity and input-finiteness gate](./docs/decisions/029-validator-anti-ambiguity.md)

### Mission-layer

- [ADR-006 — Read the 4×3 mosaic with a camera at a line-anchored stop](./docs/decisions/006-mosaic-via-camera.md) (Refined by ADR-037)
- [ADR-007 — Manipulator exposes control primitives only until the mechanism is built](./docs/decisions/007-manipulator-control-primitives-only.md) (**Superseded by ADR-032**)
- [ADR-008 — Greedy score-per-second scheduling + 2-minute time guard](./docs/decisions/008-mission-scheduling-and-time-guard.md) (Phase 4)
- [ADR-009 — Three named failure modes get explicit bounded recovery](./docs/decisions/009-failure-recovery-first-class.md) (Phase 4)
- [ADR-010 — Lightweight in-memory logging only — `EventTrace` + Serial heartbeat](./docs/decisions/010-lightweight-logging-only.md)
- [ADR-025 — Tasks layer composes subsystem calls as blocking method bodies; no JSON authoring](./docs/decisions/025-tasks-as-blocking-methods.md)
- [ADR-026 — `Tasks.runTask` is non-reentrant — nested dispatch is rejected](./docs/decisions/026-tasks-runtask-non-reentrant.md)
- [ADR-032 — Manipulator API: verb-level builders over ManipServo + ManipBelt actuator classes](./docs/decisions/032-manipulator-api-pattern.md)

### Drive-layer

- [ADR-011 — Static motor-bias scalar (Strategy B); C/D on the shelf](./docs/decisions/011-motor-bias-strategy-b.md)
- [ADR-012 — Fluent builder for any verb with 3+ knobs](./docs/decisions/012-fluent-builder-3plus-knobs.md)
- [ADR-013 — `LineFollower.setEnabled(false)` does not implicitly brake](./docs/decisions/013-linefollower-disable-no-implicit-brake.md)
- [ADR-014 — T-shape trapezoidal velocity profile, acceleration-based, opt-in](./docs/decisions/014-tshape-trapezoidal-velocity-profile.md)
- [ADR-015 — Drive owns a FIFO command queue; destructors enqueue](./docs/decisions/015-drive-command-queue.md)
- [ADR-016 — Drive builders block by default; `.async()` opts out](./docs/decisions/016-sync-default-builders.md)
- [ADR-017 — `DriveMode::SYNC` + `tShape` are the project-wide default](./docs/decisions/017-sync-plus-tshape-default-profile.md) (Refined by ADR-034)
- [ADR-018 — LineFollower distance-gated stops use symmetric trapezoidal + active brake](./docs/decisions/018-linefollower-distance-gated-stops.md) (Refined by ADR-023)
- [ADR-022 — Dual motor profile via per-motor structs + ActiveMotor alias](./docs/decisions/022-dual-motor-profile.md)
- [ADR-023 — Pass-through collinear intersections + runtime overshoot-mode probe](./docs/decisions/023-pass-through-collinear-intersections.md)
- [ADR-027 — `Drive.stopAtLine()` reads the line-sensor leaf; re-anchors localization; arm-window rule; reverse supported (2-board) via `StopAtLinePolicy` unified resolver](./docs/decisions/027-drive-stop-at-line.md)
- [ADR-034 — `.flat()` is the named opt-out from the default trapezoidal profile](./docs/decisions/034-flat-opt-out-tshape.md)
- [ADR-035 — Two-sided pivot compensation: `delta = (T/2)*tan(theta/2)` stored on the pivot edge; no global `EDGES` mutation](./docs/decisions/035-two-sided-pivot-compensation.md) (**Superseded by ADR-036**)
- [ADR-036 — Per-turn pivot compensation: a pivot is a 3-node turn `(u,v,w)`; sparse `PivotTurn[]` table; one firmware lookup resolves wheel + trim](./docs/decisions/036-per-turn-pivot-compensation.md) (extended by ADR-044)
- [ADR-038 — Blind→projection auto stop-at-line: auto-rule via `auto_rule_active`; per-edge `StopAtLineMode` override; field-sourced `Routes::BLIND_PROJECTION_AUTO_SEEK` toggle; soft-fallback default off](./docs/decisions/038-blind-projection-auto-stop-at-line.md) (soft-fallback sub-decision superseded by ADR-039)
- [ADR-039 — Blind→projection auto stop-at-line fires a normal seek (default margin); a missed line is a hard `DRIVE_FAULT`; the soft-fallback (`soft_fallback`, `_activeSoftFallback`, `STOP_AT_LINE_FAULT_FALL_BACK`) is removed; come-back recovery deferred](./docs/decisions/039-stop-at-line-normal-seek-no-soft-fallback.md)
- [ADR-044 — STOP_AT_LINE-incoming pivot trim: a seek into a pivot reduces its overshoot by δ so the axle lands δ-short; shared `resolveSensorOvershootTrim` (C++/sim); validator budgets the `"sensor"`-incoming side; δ>offset hard-rejected (reverse re-seat → #75)](./docs/decisions/044-stop-at-line-incoming-pivot-trim.md) (extends ADR-035/036 regime matrix)
- [ADR-045 — IMU yaw sign normalization: all raw IMU yaw I/O flows through `YawSource`; `Config::IMU_YAW_SIGN` applied once on read + once on STM32 gyro-heading write; turn-sign math in host-tested `TurnSignPolicy.h`](./docs/decisions/045-imu-yaw-sign-normalization.md)
- [ADR-046 — Projection is a node property, not an edge type: `NodeType{LINE,BLIND,RESET,PROJECTION}`; edges are LINE/BLIND only; per-hop primitive from `(edge_type, dest_node_type)`; auto-seek is a local destination-node read; `line–proj–line` pass-through fires off the distance gate](./docs/decisions/046-projection-as-node-property.md) (supersedes the ADR-038/039 auto-rule *trigger* sub-decision; amends ADR-023, ADR-044; retires the implicit projection-as-edge convention)

### Perception

- [ADR-019 — Custom 8-channel `WROLineArray` replaces vendor `MatrixLineTracer`](./docs/decisions/019-8channel-custom-line-sensor.md)
- [ADR-020 — Freeze LineFollower PID output during any intersection](./docs/decisions/020-junction-pid-freeze.md)
- [ADR-021 — Camera is the sole color sensor — no dedicated `ColorReader` module](./docs/decisions/021-camera-as-sole-color-sensor.md) (Refined by ADR-037)
- [ADR-024 — Bidirectional line following — runtime auto-direction + 3-layer override](./docs/decisions/024-bidirectional-line-following.md)
- [ADR-037 — HUE12 Vision capture-once latch + on-device hue classification](./docs/decisions/037-hue12-vision-capture-once.md)

### Tooling & testing

- [ADR-030 — Host-compile harness links the real production symbols; CI-enforced via the `host-tests` ctest job](./docs/decisions/030-harness-links-real-sources-ci-enforced.md)
- [ADR-031 — Config split per domain: umbrella Config.h + per-domain sub-headers + ConfigChecks.h last](./docs/decisions/031-config-split-per-domain.md) (Refined by ADR-040)
- [ADR-033 — CTPS ingests a derived mission trace recorded from the real C++; no Python re-implementation of task bodies](./docs/decisions/033-ctps-derived-mission-trace.md) (supersedes the ADR-025 "Python mirror of each task body" sim-parity sub-decision)
- [ADR-040 — Finish the per-domain Config split (config/ folder + centralized checks)](./docs/decisions/040-config-split-finished.md)
- [ADR-048 — Reset nodes are blind-incident only (a LINE edge into a `"reset"` node is rejected, like `"blind"`; projection stays legal); `primitive_mode=STOP_AT_LINE` + `stop_at_line='off'` is a rejected contradiction](./docs/decisions/048-reset-blind-incidence-and-stop-at-line-contradiction.md) (extends ADR-029 gate; mirrors ADR-046 projection-legal carve-out)
- [ADR-049 — Host-test mock HAL as the shared trust boundary: one `hal_mocks/` dir, thin record-and-test-set fidelity, no multi-tick FSM modeling, dedicated-dir shadowing rule, per-header "Fidelity" docblock](./docs/decisions/049-host-test-mock-hal.md)
- [ADR-050 — PathTracker sim↔firmware FSM parity: live trace-diff CI job (no committed golden); `EdgeDispatchPolicy` terminal classifier; single-sourced + gated `routes_fixture` (reverses the #91 freeze, re-couples to `Config.h`); 3-place trace schema; three-bucket compare; cumulative-pose cross-check; self-auditing allowlist; read-only parity accessors](./docs/decisions/050-pathtracker-sim-parity.md)

## Status summary

| Status | Count | ADRs |
|---|---|---|
| Implemented | 46 | 001, 002, 003, 004, 005 (amended by 042, 043), 006 (refined by 037), 010, 011, 012, 013, 014, 015, 016, 017 (refined by 034), 018 (refined by 023), 019, 020, 021 (refined by 037), 022 (partially superseded by 047), 023 (amended by 046), 024, 025 (sim-parity sub-decision superseded by 033), 026, 027, 028, 029, 030 (all listed surfaces linked from production; `reset_arrival` + `return_to_node_dispatch` de-mirrored, #113 landed — zero mirrored PathTracker FSM logic remains; the Drive wall-stop watchdog mirror in `test_wall_stop_watchdog` is the last inline mirror), 031 (refined by 040), 032, 033, 034, 036 (extended by 044), 037, 038 (soft-fallback sub-decision superseded by 039; auto-rule trigger sub-decision superseded by 046), 039 (auto-rule trigger sub-decision superseded by 046), 040, 041, 042 (amends 005 "never stored" clause — parent stays Implemented), 043 (amends 005 "yaw written only by zeroHeadingFromCurrent" clause — parent stays Implemented), 044 (extends the 035/036 pivot regime matrix; amended by 046), 045, 046 (supersedes the 038/039 auto-rule trigger sub-decision; amends 023, 044), 047 (partially supersedes 022), 048 (extends 029; mirrors 046), 049, 050 (supersedes the issue-#91 fixture freeze/decouple convention — no prior numbered ADR) |
| Partially implemented | 2 | 008 (Mission DSL shipped; greedy scheduler + time guard deferred to Phase 6), 009 (lost-line + missed-intersection in Phase 1/2; failed-grip pending manipulator) |
| Pending implementation | 0 | — |
| Deferred | 0 | — |
| Superseded | 2 | 007 (superseded by ADR-032 — manipulator verb-level API now that CAD is done); 035 (superseded by ADR-036 — per-turn pivot model replaces the per-edge `pivot_trim_cm` field). Sub-decision supersessions (parent ADR stays Implemented): ADR-010 supersedes the Phase 0 "no logging at all" stance; ADR-033 supersedes the ADR-025 "Python mirror of each task body" sim-parity sub-decision; ADR-039 supersedes the ADR-038 soft-fallback sub-decision (the auto-rule / per-edge override / toggle survive); ADR-046 supersedes the ADR-038/039 auto-rule *trigger* sub-decision (projection became a node type — `auto_rule_active` is now a local destination-node read, not the route-relative `next_hop_is_projection` peek; the ladder / override / toggle / hard-fault posture survive) and retires the implicit projection-as-edge convention. |

## Conventions

- ADR numbers are assigned **in order of first appearance**, not by topic. Never renumber.
- When a decision changes, write a new ADR with the next number, mark the old one **Superseded by ADR-NNN**, and update both the "Supersedes" / "Superseded by" cross-links.
- Each ADR stays one self-contained page (~30–60 lines). If a decision genuinely needs more, link out to a design doc under `docs/`; do not let the ADR grow.
