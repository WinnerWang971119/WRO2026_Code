# Blind-run coalescing: one braked move per collinear blind run

**Status:** Implemented in software; physical bench acceptance remains pending.
**Supersedes the approach in:** `docs/plans/2026-07-22-blind-column-overshoot.md` (the carry+stub-guard fix — bench-proven insufficient; see Decisions).

**Goal:** A dead-reckoned collinear blind run (e.g. `goTo("MWU")` up the mosaic column) lands its final node within <1 cm instead of overshooting into the wall, by driving the whole run as **one** braked `driveDist` instead of N per-hop moves.

**Approach:** Bench-confirmed root cause: with per-hop chaining, the chassis cruises through the collinear run at ~70 PWM and the final short hop (6–9 cm) has nowhere near enough runway to decelerate to the floor before braking, so it brakes from speed and overshoots (a 9.3 cm `goTo("MWD")` overshoots too — the real chassis decelerates slower than the profile's `3.5 PWM/cm` assumes). A **single combined move** over the full ~70 cm run has the entire run as deceleration runway — exactly the geometry of the hand-measured 100 cm move that landed at 99.98 cm — and, having no inter-hop boundaries, also has **no boundary carry to accumulate**. So one move fixes both the deceleration and the carry at once. This reopens ADR-059's rejected run-coalescing: its rejection rested on parity-granularity cost plus a physics claim ("short combined runs won't land <1 cm") that the bench has now refuted (short single moves land accurately at any length on this chassis).

## Scope

- **In scope:**
  - **Firmware:** when the walk-ahead finds a strictly-collinear BLIND run of ≥2 hops, dispatch ONE `Drive.driveDist(totalCm).setMode(runMode).tShape()` (braked) instead of the per-hop `ChainSpeedPolicy`-planned moves; on completion, advance `Localization` through every intermediate node to the final node.
  - **Remove the now-subsumed per-hop machinery:** revert/remove the committed T4 stub-guard (`2513cc2`), the T5 generalized-carry state machine (`0d78996`), and their host tests T6 (`796cedc`); and remove the ADR-059 per-hop blind-chain velocity plan + measured-carry subtraction for blind runs (the `ChainSpeedPolicy`-driven blind dispatch, the in-chain AC9 carry, the degraded stub) to the extent they are blind-only. (Keep whatever `ChainSpeedPolicy` serves LINE/PROJECTION chains — verify usage; see Notes.)
  - **Sim / parity / CTPS mirrors:** `tools/path_planner/sim/runtime.py`, the parity trace schema (`parity.py` + `PathTrackerParityTrace.h`), and the CTPS run-grouper (`track_builder.py`) mirror "one combined move advancing through N nodes."
  - **Host tests:** replace the blind-chain per-hop tests with coalescing tests (one `driveDist(total)` dispatched; Localization advances through all nodes; run lands at the final node), asserting the REAL `MiniR4.DriveDC` setpoint.
  - **ADR-080** superseding ADR-079 (carry/stub removed) and reopening ADR-059's coalescing rejection (its own "revisit if the tradeoff shifts" trigger fired).
- **Out of scope:**
  - LINE / PROJECTION collinear pass-through (they re-seek the line at each node and self-correct; unchanged).
  - The 28 pre-existing branch test failures (bench-tuning golden/mirror/pin drift — separate re-sync; documented in the superseded spec).
  - GYRO_HOLD gains (kept, intentional bench tune) and `MISSION_ALLOW_REARM_DEFAULT` (leave `true`).
  - Any drive-constant retune.

## Decisions

- **Coalesce, don't patch per-hop.** The per-hop carry+stub approach (superseded spec) treats symptoms of the per-hop structure; one combined move removes the structure. It's the only approach that gives the final approach a full deceleration runway AND eliminates the carry — bench-confirmed necessary (both chaining states overshoot; `goTo("MWD")` on 9.3 cm overshoots).
- **Drop T4/T5/T6.** After coalescing, no braked-blind scenario exercises the per-hop carry or the chain-final stub (collinear runs coalesce; turn-separated hops start from rest and land accurately). T5 also carries a latent bug (a non-chain hop shrunk <5 cm slams flat from cruise). Reverting them is strictly better than keeping dead+buggy code. Keep T1/T2/T3.
- **Coalescing is stricter than the retired blind chain.** Every admitted hop must share hop 0's resolved drive mode and speed tier, pass the pairwise `NAV_TURN_TOLERANCE_DEG` gate, and keep its predicted endpoint strictly within `PATH_BLIND_COALESCE_MAX_POSITION_ERROR_CM` (1 cm) of hop 0's execution ray. A mode/speed boundary, real turn, or cumulative geometric miss splits the run.
- **Localization belief is updated at run completion** (advance through all nodes then), accepting that mid-run belief is stale for the run's duration — acceptable for a dead-reckoned run with no mid-run correction (WRO freezes on fault; there is no mid-hop recovery anyway — see memory `gotcha-localization-no-inhop-pose`).
- **Reuse the walk-ahead.** `PathTracker::_chainTryStart` already finds the collinear run (ordered nodes `_chain.node[]`, per-hop distances `_chain.hopDistCm[]`, `hopCount`, total). Coalescing consumes that; it does not re-derive routing.
- **Gate + caps unchanged:** reuse `Config::PATH_BLIND_CHAIN_ENABLED` (already re-enabled by T1) as the on/off, `MAX_CHAIN_HOPS` + `PATH_BLIND_CHAIN_MAX_CM` as the coalesced-run caps, `NAV_TURN_TOLERANCE_DEG` as the collinearity gate. (Optionally rename to `PATH_BLIND_COALESCE_*` in the ADR; not required.)

## Acceptance Criteria

- [ ] AC1 — Bench: `goTo("MWU")` from `i2` (chaining/coalescing on) lands within **<1 cm** of MWU, clear of the top wall. AND the return descent `MWU→…→MYD` lands each within <1 cm.
- [x] AC2 — Host ctest: a collinear blind run of N≥2 hops dispatches **exactly one** `driveDist(totalCm)` (not N per-hop `MoveSync`/`MoveSyncDegs` calls), braked, at `setMode(runMode)` — asserted against the real `MiniR4.DriveDC` mock (`lastMoveSyncL/R`, `lastBrake`, and the dispatch call-count == 1 for the run).
- [x] AC3 — Host ctest: on completion of that one move, `Localization.getCurrentNode()` == the run's final node, having advanced through every intermediate node (verify the node stamp sequence, e.g. via the parity `blind_pass` records or a Localization observer).
- [x] AC4 — Host ctest: the combined move commands `totalCm` == sum of the run's hop distances (no per-hop carry subtraction; parked-encoder host build lands exactly on the final node).
- [x] AC5 — A run broken by a real turn (> `NAV_TURN_TOLERANCE_DEG`) is NOT coalesced across the turn: two collinear segments, each its own combined move, the turn a normal braked stop-and-turn between them.
- [x] AC6 — `PATH_BLIND_CHAIN_ENABLED` off restores today's per-node behavior (or the pre-coalescing behavior) exactly.
- [x] AC7 — Sim↔firmware parity green: `runtime.py` + parity trace + CTPS mirror the coalesced run (one motion record + per-node `blind_pass`), no unmirrored divergence. The affected suites pass; repository-wide verification retains only the unrelated baseline failures (16 pytest and 6 ctest on 2026-07-22).
- [x] AC8 — T4 (`2513cc2`), T5 (`0d78996`), T6 (`796cedc`) reverted/removed; no `_brakedBlindCarry*` / stub-guard code remains; `git grep` clean.
- [x] AC9 — ADR-080 written (supersedes ADR-079; reopens ADR-059's coalescing rejection); ADR-079 marked Superseded by ADR-080; ADR-059 cross-linked; DECISIONS.md index + status table updated.

## Design / Contracts

### Firmware — coalesced dispatch (`PathTracker.cpp`)

Before implementation, the blind-dispatch arm (`_dispatchEdgeAfterTurn`, `BLIND_DEAD_RECKON`) dispatched per-hop moves carrying `ChainSpeedPolicy` start/end PWM. The implemented blind-run path now does the following:

1. **Detect the coalesced run.** When `_chainTryStart` succeeds (a collinear blind run of `hopCount ≥ 2`), compute `totalCm = Σ _chain.hopDistCm[0..hopCount-1]` (the walk-ahead already accumulates this) and the resolved `runMode` (GYRO here; per-run, from the edges' resolved mode) and body heading.
2. **Dispatch ONE braked move** (RAII single-statement, `.async()`):
   `Drive.driveDist(totalCm).setMode(runMode)[.setHeading(imuTarget)].setStartSpeed(entrySeed).tShape().async();`
   - `entrySeed` = the ADR-023 captured cruise if the run opens right after a pass-through, else the cold-start `TSHAPE_START_SPEED`.
   - Default `endSpeed=0` + brake (the ordinary `cruise→floor→brake` stop — now with the full run as ramp room, so the shed fits trivially).
3. **On arrival** (`Drive.isIdle()` in `State::BLIND_FORWARD`): advance `Localization` through every node of the run — `for k in 1..hopCount: Localization.onDeadReckonedAdvance(_chain.node[k])` — then settle/plan. Emit the parity records (below) in the same order the advance happens.
4. **Remove** the per-hop `_chain.speed[]` velocity plan, the in-chain AC9 measured-carry subtraction, and the degraded stub for the blind path (all subsumed). Keep the walk-ahead's run-finding.

### Firmware — remove T4/T5 (revert)

Revert commits `2513cc2` (T4 stub-guard), `0d78996` (T5 generalized carry), `796cedc` (T6 tests). Simplest: `git revert --no-commit 796cedc 0d78996 2513cc2` then reconcile against the coalescing edit (both touch `PathTracker.cpp`), OR remove the `_brakedBlindCarry*` members + `_measuredBoundaryCarryCm` + the stub floor-pin by hand. End state: no `_brakedBlindCarry*`, no `_firstHopOfGoTo`, no stub-guard.

### Localization — multi-node advance

`onDeadReckonedAdvance(node)` stamps `_currentNode` (single-writer, ADR-005). Advancing through N nodes = calling it N times in run order at completion. If a cleaner API is wanted, add `Localization::advanceThrough(const NodeId* nodes, uint8_t count)` that loops internally (still single-writer). The chassis is physically at the final node when this runs, so the belief jump is correct.

### Trace / parity (`PathTrackerParityTrace.h`, `sim/parity.py`)

Represent the coalesced run as **one** motion record + one `blind_pass` per intermediate node:
- Bump the strict contract to parity schema v4 because the motion record's required field set changes.
- Emit ONE `blind_dead_reckon` record for the whole run: `node = finalNode`, `dist_cm = totalCm`, `cruise_pwm = resolvedCruise`, `start_pwm = entrySeed`, `end_pwm = 0`, and `chain: true`.
- Emit a `blind_pass {node}` for each intermediate node (nodes `1..hopCount-1`); the final node is the braked arrival (no `blind_pass`).
- Firmware and sim must emit byte-identical records (the ADR-050 strict schema faults on either side landing alone). Update `parity.py`'s field spec + `_EXACT_FIELDS`/bucketing if a field changes.

### Sim (`runtime.py`)

`PathTrackerSim._try_start_blind_chain` + the `_walk_hops` blind dispatch: model ONE combined forward advance of `totalCm` (`self._chassis.forward(totalCm)`), then `on_dead_reckoned_advance(node)` for each node in run order; yield the one motion primitive + per-node `blind_pass`. The chassis lands on the final node (drift 0, per `gotcha-sim-chassis-axle-on-node`). Mirror the firmware's coalesce/no-coalesce decision exactly (`feedback-sim-mirrors-robot`).

### CTPS (`track_builder.py`)

The run-grouper already merges a collinear run into one trapezoid (ADR-059). Update the loader/`track_builder` to accept the new one-record-per-run trace (the `_KNOWN_DRIVE_METHODS`/`blind_pass` handling). Likely small since the render already treats the run as one profile — verify by rendering the column and eyeballing (localhost, per `gotcha-eyeball-ctps-html-via-localhost`).

### Contracts summary (cross-surface — must stay byte-identical firmware↔sim)

- One `blind_dead_reckon {node=finalNode, mode, dist_cm=totalCm, cruise_pwm=resolvedCruise, start_pwm=entrySeed, end_pwm=0, <run flag>}` per collinear blind run.
- One `blind_pass {node}` per intermediate node, in run order, before the final arrival.
- Localization advances i2→…→final, once, at arrival.

## Testing Strategy

| ID | Test | Type | Expected |
|----|------|------|----------|
| TC1 | N≥2 collinear blind run | Host | exactly ONE `driveDist(totalCm)` dispatched (call-count==1), braked, `runMode` (AC2) |
| TC2 | same run completion | Host | `getCurrentNode()==final`, node stamps hit every intermediate (AC3) |
| TC3 | commanded distance | Host | == Σ hop distances; parked-encoder build lands exactly on final (AC4) |
| TC4 | run split by a real turn | Host | two coalesced segments + a braked turn between (AC5) |
| TC5 | `PATH_BLIND_CHAIN_ENABLED` off | Host | pre-coalescing behavior (AC6) |
| TC6 | parity trace diff (fw vs sim) | pytest | one motion + per-node pass, byte-identical, green (AC7) |
| TC7 | full ctest + full pytest | both | only the unrelated baseline failures remain (AC7) |
| TC8 | Bench: `goTo("MWU")` up + descent | Bench | land <1 cm each node (AC1) — the real gate |

Host mock surface: `MiniR4.DriveDC.lastMoveSyncL/R`, `lastBrake`, dispatch call-count; the `test_path_tracker_blind_chain.cpp` harness + fixtures. Rewrite that file's per-hop chain expectations to coalescing expectations. Run: `ctest --test-dir build/test`; `uv run --extra dev pytest tools/path_planner/tests/`.

## Tasks (suggested order — firmware first so the robot works, mirrors second for CI)

| ID | Task | Risk | Files |
|----|------|------|-------|
| C1 | Revert T4/T5/T6; confirm no `_brakedBlindCarry*`/stub code remains | med | `PathTracker.cpp`/`.h`, `test_path_tracker_blind_chain.cpp` |
| C2 | Firmware: coalesced dispatch (one `driveDist(total)`) + Localization multi-node advance; remove blind per-hop velocity/carry/stub | **high** | `PathTracker.cpp`/`.h`, `Localization.*` (if `advanceThrough` added) |
| C3 | Host ctests TC1-TC5 (real setpoint; coalescing behavior) | high | `test_path_tracker_blind_chain.cpp` (+ CMake if new file) |
| C4 | Parity trace: one-record-per-run + per-node `blind_pass` (firmware emit + `parity.py` + `PathTrackerParityTrace.h`) | high | `PathTrackerParityTrace.h`, `sim/parity.py`, `PathTracker.cpp` emit |
| C5 | Sim `runtime.py`: mirror coalesced advance + emit | high | `tools/path_planner/sim/runtime.py` |
| C6 | CTPS loader/grouper accepts the new trace; eyeball the column render | med | `tools/path_planner/ctps/track_builder.py`, `trace_loader.py` |
| C7 | ADR-080 (supersede ADR-079, reopen ADR-059 coalescing); DECISIONS.md; mark ADR-079 Superseded | low | `docs/decisions/080-*.md`, `079-*.md`, `059-*.md`, `DECISIONS.md` |
| C8 | Full ctest + full pytest + parity; confirm only the 28 pre-existing failures remain | low | run-only |

C2 is the robot fix (bench-testable after C1+C2+a flash). C4/C5/C6 are the parity/CI mirror (the four-surface fan-out — do NOT skip or the sim silently lies; `gotcha-direction-contract-four-surface-mirror`).

## Notes for Implementer

- **The bench is the arbiter (AC1).** C1+C2 alone are flashable — verify `goTo("MWU")` lands before investing in C4-C6.
- **Verify `ChainSpeedPolicy` usage before deleting it.** If it serves only blind chains, its dispatch path, `chain_speed_dump`/goldens, and `chain_speed.py` can retire with the blind chain; if it also serves LINE/PROJECTION chaining, keep it for those and only bypass it for blind runs. Grep `resolveChainSpeeds` / `_chain.speed`.
- **Trace-invisible dispatch is a known trap** (`gotcha-parity-ctps-record-plan-not-dispatch`): the parity trace records the plan, so a host ctest MUST assert the REAL dispatched `Drive` setpoint (one `driveDist(total)`), not just the trace.
- **Four-surface mirror discipline** (`gotcha-direction-contract-four-surface-mirror`, `feedback-sim-mirrors-robot`): change firmware + `runtime.py` + parity + CTPS together; a parity test only catches drift if it drives the coalesced case — add a collinear-blind-run fixture that the parity sweep traverses (`gotcha-parity-job-field-source`: author on a swept `field_fixture.py` edge, regenerate `routes_fixture`).
- **RAII builders** stay single discarded-temporary `.async()` statements (`gotcha_drive_builder_raii`).
- **Real-field residuals** (`gotcha-axis-aligned-fixtures-hide-subtolerance-turns`): the collinearity gate must use `NAV_TURN_TOLERANCE_DEG` (10°), not a tighter epsilon, or near-collinear real-field runs mis-split.
- **`SYNC` is broken on this chassis** (crossed encoder cables); blind runs are GYRO — the combined move must use the run's resolved mode, not force SYNC.
- **Localization mid-run staleness** is accepted (Decisions); do not add a mid-move distance observer (that was ADR-059's stated coalescing cost — we accept it, not solve it).
- **Provenance:** this reopens ADR-059's run-coalescing (rejected on parity-granularity + a now-refuted physics claim). The prior carry+stub fix (`docs/plans/2026-07-22-blind-column-overshoot.md`, ADR-079) is bench-proven insufficient: with chaining on, the final short hop enters at cruise (~67) and can't decelerate in 6–9 cm (even a 9.3 cm `goTo("MWD")` overshoots), because the real chassis decelerates slower than `TSHAPE_ACCEL_PWM_PER_CM=3.5` assumes; T4 only lowers the commanded PWM and can't shed the entering momentum, and T5 doesn't act in-chain. A single ~70 cm move has the runway (100 cm move = 99.98 cm) and no carry.
