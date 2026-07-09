# Collinear-Run Continuous Motion (whole-run trapezoid + CTPS truth)

**Goal:** Make a `goTo()` over a collinear chain of nodes drive as ONE continuous motion — a single ramp-up at the start, cruise through the intermediate nodes, and a single ramp-down over the end — instead of stopping and re-accelerating at every node; and make the CTPS player render that truthfully.

**Approach:** Reframed after a cross-vendor architecture consult (Fable read the code; Codex reasoned from spec) — both vetoed the original "whole-run distance accounting inside LineFollower" mechanism. Three findings drive the new shape: (1) **LINE/projection collinear chains already produce the whole-run trapezoid in firmware** when pass-through fires — the stop is `cruise→floor(50)→brake` (ADR-058), so the shed-to-floor is only `(85−50)/5 = 7 cm`, which fits the 12 cm final overshoot window; no cross-hop ramp arming is needed. (2) **CTPS is already lying** about shipped behavior — `ACT_LINE_PASS` sits in `_STOP_GATED_TAGS`, so CTPS renders a full stop at every node the firmware cruises through. (3) **BLIND/dead-reckon chains are the only place needing new firmware code** — they stop at every node today, and the fix is standard **backward-pass velocity chaining** (the GRBL/Marlin junction-velocity scheme): keep one command per hop, but chain `setStartSpeed/setEndSpeed/setBrake(false)` so the whole-run trapezoid — including a ramp-down that spans hop boundaries — falls out with **no new cross-hop controller state**. The per-hop chain-speed math lives in a header-only `ChainSpeedPolicy.h` (the ADR-058 policy pattern), mirrored once in Python and pinned by golden vectors; the sim *emits* the resolved per-hop `(start_pwm, end_pwm)` into the parity/trace record and CTPS *consumes* them — CTPS never re-derives a chain decision.

## Scope

- **In scope:**
  - CTPS truth-fix: a run-grouper post-process in `track_builder.py` that merges consecutive pass-tagged primitives into ONE whole-run velocity profile, so a pass-through run renders as one trapezoid (line/projection first — shippable standalone; blind added later in the same plan).
  - `ChainSpeedPolicy.h` (firmware, header-only, pure) + a Python mirror + cross-language golden vectors: the backward-pass per-hop `(start_pwm, end_pwm)` math.
  - Firmware `PathTracker` blind-chain chaining: detect a collinear blind run, dispatch its hops with chained start/end speeds and `setBrake(false)` on the mid-hops so the run is one continuous motion; the final hop brakes.
  - Guards for blind chaining: a `PATH_BLIND_CHAIN_ENABLED` kill-switch (default ON), a `PATH_BLIND_CHAIN_MAX_CM` run-distance cap, and ineligibility for pivot / seek / reset / goal nodes.
  - Sim mirror (`runtime.py` / `parity.py`): the blind-pass primitive + chain-eligibility decision + the additive `(start_pwm, end_pwm)` parity fields.
  - A `static_assert` closing the one real LINE-chain edge case Finding 1 leaves (FAST preset + REAR-led final hop: `(90−50)/5 = 8 cm > 7.68 cm` rear axle offset).
  - New ADR (amends ADR-023/046/052/058) + tests (host ctest, CTPS/sim pytest, parity sweep, golden vectors, a new blind-chain fixture).

- **Out of scope:**
  - **`NAV_TURN_TOLERANCE_DEG` stays 0.5°** — no tolerance change. Diego does not want a shallow real turn silently skipped; he will author the field nodes truly collinear (within 0.5°) himself, in parallel. No new dedicated pass-through tolerance constant.
  - **No `field_data.json` edits by us** — Diego owns the real-field / test-field node geometry (the CY1→CY3 columns are his to straighten). We validate the feature with the existing `collinear_4node` fixture + a new blind-chain fixture, NOT by editing the CY columns.
  - Run-*coalescing* into a single multi-hop Drive command (rejected: breaks the hop-granular parity trace / ADR-050 and needs a mid-move distance observer).
  - Any new mid-run positional re-anchor for blind chains (blind nodes have none today; we do not invent one — see Decisions).
  - Mission/task authoring changes; the manipulator; the real competition mission content.

## Decisions

- **Reframed mechanism (both consultants vetoed the original):** no run-scoped ramp-down distance accounting inside `LineFollower`/`Drive`. Line chains already work; blind chains use per-hop backward-pass chaining with no cross-hop controller state (ADR-050 warns against widening cross-hop state into the low-level controllers). — *Fable + Codex, corroborated.*
- **CTPS-truth-first, standalone:** ship the CTPS run-grouper fix (line/projection) as an independent, low-risk change before the firmware blind work. It repairs an already-shipped falsehood and honors Diego's "CTPS tells the truth of the firmware" principle. — *Both consultants.*
- **Blind chains: IN, and high priority.** Diego's bench experience: every stop→launch is a static-friction break-away that spikes heat and slips the wheels, corrupting the next move's heading; on blind/dead-reckon hops there is no line to wash it out, so a continuous run dead-reckons BETTER than N stop-starts. Blind nodes have NO positional re-anchor today, so stopping at them corrects nothing — removing the stop loses nothing and gains repeatability. (memory: `project-fewer-stops-better-friction-drift`.)
- **Guarded, not gated-off:** blind chaining defaults ON but behind `PATH_BLIND_CHAIN_ENABLED` (rollback kill-switch, the `PATH_PASS_THROUGH_ENABLED` precedent) and a `PATH_BLIND_CHAIN_MAX_CM` cap; ineligible at pivot/seek/reset/goal nodes. Bench verification is Diego's ("then I'll try").
- **Level 2 falls out of the backward pass:** `v_entry(node_k) = min(cruise, PWM_MIN + accel × dist(node_k → run_end))` sets each node's entry speed low enough to still shed to the floor by the run end, so a short final hop pulls the ramp-down back into the previous hop — a true whole-run trapezoid with hop-spanning ramp-down, no distance accounting in the controller.
- **Ramp-down is 7 cm (shed-to-floor), not 17 cm.** The physical stop is `cruise→PWM_MIN(50)→active-brake` (ADR-058), so the decel that must fit is `(cruise−PWM_MIN)/accel`, not `cruise/accel`. This is why line chains already fit.
- **Parity by data + goldens, not triplicated logic:** the speed math is transcribed exactly twice (`ChainSpeedPolicy.h` + Python mirror), golden-pinned; the sim emits the resolved `(start_pwm,end_pwm)` and CTPS consumes them. The chain-*eligibility* predicate reuses `_passThroughHeadingOk` and is policed by the existing live sim↔firmware trace-diff (ADR-050) via all-pairs `goTo` sweeps.
- **Keep the single tolerance knob at 0.5°.** Splitting a dedicated pass-through tolerance was considered (Codex) and rejected (Fable): pass-through ⟺ skipped-turn is definitional, so a split where pass-through fires while the pre-edge turn would still dispatch is incoherent. Diego chose to keep it tight and straighten the field.
- **New ADR-059** amends ADR-023 (pass-through), ADR-046 (projection node), ADR-052 (heading re-anchor), ADR-058 (CTPS motion profile). Verify no ADR-number collision at execute time (memory: `gotcha-adr-number-collision`).

## Acceptance Criteria

- [ ] AC1 — On a genuinely collinear LINE chain (the `collinear_4node` fixture), CTPS renders ONE trapezoid: a single ramp-up at the run start, constant cruise through the intermediate nodes (no dip to floor, no stop keyframes), a single ramp-down/stop at the end. (Before this change it renders a stop at each node.)
- [ ] AC2 — On a collinear BLIND chain (new fixture), the firmware dispatches the run with chained speeds: mid-hops carry `setStartSpeed`/`setEndSpeed`/`setBrake(false)` and the chassis never brakes to rest at an intermediate node; only the final hop brakes. Each intermediate node is still stamped exactly once (`onDeadReckonedAdvance`).
- [ ] AC3 — The blind-chain velocity profile is a true whole-run trapezoid: when the final hop is shorter than the shed-to-floor distance, the ramp-down begins in the previous hop (verified by the per-hop `(start_pwm,end_pwm)` from `ChainSpeedPolicy`).
- [ ] AC4 — `ChainSpeedPolicy.h` (C++) and its Python mirror produce byte-identical `(start_pwm,end_pwm)` on the golden vectors (float32/int16 discipline, the ADR-058 pattern).
- [ ] AC5 — Guards work: with `PATH_BLIND_CHAIN_ENABLED=false`, a blind chain stops at every node (today's behavior); a run exceeding `PATH_BLIND_CHAIN_MAX_CM` stops at the node where the cap trips; a pivot/seek/reset/goal node ends the chain (that hop brakes).
- [ ] AC6 — The sim↔firmware FSM parity job (ADR-050) is GREEN over an all-pairs `goTo` sweep that includes collinear line and blind chains — the two surfaces agree on every primitive, direction, and the new `(start_pwm,end_pwm)` fields.
- [ ] AC7 — CTPS consumes the recorded `(start_pwm,end_pwm)` for blind runs and renders their whole-run trapezoid; it never re-derives a chain decision. The frozen player-asset-pin tests stay green (or are updated once, intentionally).
- [ ] AC8 — A `static_assert` fails the build if a config retune makes the FAST + REAR-led final-hop shed exceed the rear overshoot window (the one LINE-chain edge Finding 1 leaves).
- [ ] AC9 — Full `pytest` (path-planner, CTPS, parity) and full `ctest` (host harness) pass; `arduino-cli compile` passes; regenerated `routes.h/.cpp` are unchanged (no field edits).

## Contracts & Interfaces

Single source of truth for every cross-task seam.

### Shared types / signatures

- **`ChainSpeedPolicy.h`** (firmware, header-only, pure — no Arduino surface, host-testable, the ADR-058 `TShapeProfilePolicy.h` pattern). Owner: **T3**. Consumers: T6 (firmware dispatch), T4 (Python mirror), T9 (host golden TU).
  - `struct ChainSpeedConfig { int16_t pwmMin; int16_t pwmMax; float accelPwmPerCm; };`
  - `struct ChainHopSpeed { int16_t startPwm; int16_t endPwm; };`
  - `// Backward pass over an ordered collinear run. hopDistCm[i] is hop i's length (node_i → node_{i+1}); hopCount hops, hopCount+1 nodes; the last node is the run end (stop). cruisePwm is the signed run cruise. Fills out[i] for each hop. Pure, distance-domain, float32/int16 discipline.`
  - `void resolveChainSpeeds(const float* hopDistCm, uint8_t hopCount, int16_t cruisePwm, const ChainSpeedConfig& cfg, ChainHopSpeed* out);`
  - Math (pinned): `distToEnd[k] = sum(hopDistCm[k..end])`; `vEntry[k] = clampMag(min(|cruise|, pwmMin + accel*distToEnd[k]), pwmMin, |cruise|) * sign(cruise)`; `vEntry[0] = startSeed (±TSHAPE_START_SPEED)`; `vEntry[nNodes-1] = 0` (final stop); `out[i] = { vEntry[i], vEntry[i+1] }`.
- **`chain_speed.py`** (`tools/path_planner/ctps/chain_speed.py`, NEW). Owner: **T4**. Pure float32 mirror of `resolveChainSpeeds` (numpy float32 + int16 truncation, exactly like `ctps/profile.py`'s kernel). Consumers: T7 (sim emit), T8 (CTPS), T10 (golden test).

### Primitive pass tags (the 3-place CTPS contract — memory: `gotcha-ctps-drive-method-contract`)

- Existing: `ACT_LINE_PASS`, `ACT_FOLLOW_DIST_PASS`, `ACT_PROJECTION_REVERSE_PASS` (line/projection pass-through).
- NEW: `ACT_BLIND_PASS` (blind-chain mid-hop; a `driveDist` hop that does not brake). Owner of the tag string: **T7** (sim `runtime.py`). Must be added in lock-step to all three places: sim emit (`runtime.py`), loader whitelist (`ctps/trace_loader.py` `_KNOWN_DRIVE_METHODS`/pass set), renderer/grouper (`ctps/track_builder.py`).

### Parity / trace record additive fields (ADR-050 3-place schema; memory: `gotcha-parity-record-kind-is-discriminator`)

- Add `start_pwm: int`, `end_pwm: int` to the drive/blind primitive record. Additive-optional (absent → resolver default via `resolve_effective_speed`), so manip-free / non-chain traces stay byte-identical. Owner: **T7**. Consumers: `ctps/trace_loader.py` (accept, strict int-or-none guard like `power`), `ctps/track_builder.py` (consume). An unknown field must raise `ParitySchemaError`, never be silently dropped.

### CTPS run-grouper contract

- Owner: **T2** (line/proj), extended by **T8** (blind). Rule: scan the primitive stream; a **run** is a maximal sequence where every non-final primitive carries a pass tag and the final primitive is the run's stop; build ONE velocity profile over the run (line/proj: summed chord distance via `resolve_drive_profile`; blind: consume the recorded per-hop `(start_pwm,end_pwm)`), then slice `sample_drive` output at each primitive's cumulative-distance boundary. **Invariant (TC7):** each primitive's LAST sub-keyframe still byte-pins its sim-snapshot pose — only the timing/velocity BETWEEN pose boundaries changes.

### Config knobs (mirrored firmware ↔ sim, regex-parity like `PATH_PASS_THROUGH_ENABLED`)

- `Config::PATH_BLIND_CHAIN_ENABLED` (bool, default `true`) — `ConfigNav.h`. Sim mirror: `runtime.py` module constant + a `test_pass_through`-style regex-grep parity test.
- `Config::PATH_BLIND_CHAIN_MAX_CM` (float) — `ConfigNav.h`, the run-distance cap.
- Owner: **T5**. Consumers: T6 (firmware), T7 (sim).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `docs/decisions/059-*.md`, `DECISIONS.md` | T1 | — |
| `tools/path_planner/ctps/track_builder.py` | T2 | T8 |
| `tools/path_planner/ctps/profile.py` (grouper helper only) | T2 | T8 |
| `WRO2026/src/drive/ChainSpeedPolicy.h` | T3 | T6 |
| `tools/path_planner/ctps/chain_speed.py` (new) | T4 | T7, T8 |
| `WRO2026/config/ConfigNav.h`, `ConfigDrive.h`, `ConfigChecks.h` | T5 | T6, T7 |
| `WRO2026/src/mission/PathTracker.cpp` / `.h` | T6 | — |
| `tools/path_planner/sim/runtime.py`, `sim/parity.py` | T7 | T8 |
| `tools/path_planner/ctps/trace_loader.py` | T7 | T8 |
| `WRO2026/test/test_path_tracker_blind_chain.cpp` (new), `WRO2026/test/CMakeLists.txt` | T9 | — |
| `tools/path_planner/tests/**` (fixtures + unit/parity tests) | T10 | — |

## Data Model

- **Backward-pass state (firmware, fixed-size, no heap):** a small `PathTracker` member run buffer sized to a compile-time `MAX_CHAIN_HOPS` constant (e.g. 8), holding the walked-ahead hop distances + resolved `ChainHopSpeed[]`. Overflow (run longer than `MAX_CHAIN_HOPS` or `PATH_BLIND_CHAIN_MAX_CM`) simply ends the chain at the cap — the capped hop brakes and a fresh chain re-forms after.
- **Parity record (additive):** `{ ..., "start_pwm": int, "end_pwm": int }` on the drive/blind primitive. Absent ⇒ resolver default (unchanged for non-chain hops).

## Error Handling

- **Chain longer than `MAX_CHAIN_HOPS` / `PATH_BLIND_CHAIN_MAX_CM`:** not a fault — end the chain at the cap (that hop brakes to rest), then start a new chain. `log()`/Serial note the truncation so a silently-shortened run is visible (no silent cap — memory discipline).
- **Ineligible node mid-walk (pivot/seek/reset/goal, direction flip, non-collinear):** end the chain before it; that hop is a normal braked hop. This is the same predicate as the existing `_passThroughHeadingOk` + direction guard.
- **`ChainSpeedPolicy` degenerate inputs:** zero/negative hop distance floors to 0 (defensive, mirrors `_execProfileMove`); `accel <= 0` clamps to a tiny positive slope (mirror `resolve_drive_profile`). Never divide by zero.
- **CTPS run-grouper meets a malformed / mixed run** (a pass tag without a following stop, or a missing recorded speed): reject the group — fall back to per-primitive rendering (today's behavior) rather than guess. Emit a `UserWarning`.
- **Parity: unknown record field** → `ParitySchemaError` (loud), never silent-drop.

## Testing Strategy

**Levels:** Unit (ChainSpeedPolicy golden, CTPS grouper), Integration (host FSM ctest, sim↔firmware parity), Golden (cross-language vectors).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `ChainSpeedPolicy` C++ vs Python on golden vectors (3-hop even, short-final-hop, single-hop degenerate, reverse cruise) | Unit/Golden | Byte-identical `(start_pwm,end_pwm)`; short final hop pulls ramp-down into the prior hop |
| TC2 | Collinear 4-node LINE fixture rendered through CTPS after the grouper fix | Unit | ONE trapezoid; zero intra-run stop keyframes; last sub-keyframe of each primitive pins its snapshot pose (TC7 invariant) |
| TC3 | Collinear BLIND chain, `PATH_BLIND_CHAIN_ENABLED=true` | Integration (host ctest) | Mid-hops dispatched with `setBrake(false)` + chained start/end; no brake-to-rest at intermediate nodes; each node stamped once via `onDeadReckonedAdvance` |
| TC4 | Same blind chain, `PATH_BLIND_CHAIN_ENABLED=false` | Integration | Stops at every node (today's behavior) |
| TC5 | Blind run exceeding `PATH_BLIND_CHAIN_MAX_CM` | Integration | Chain ends at the cap hop (it brakes); a new chain re-forms after; truncation logged |
| TC6 | Blind chain whose next node is a pivot / seek / reset / goal | Integration | Chain ends before that node; that hop is a normal braked hop |
| TC7 | sim↔firmware parity all-pairs `goTo` sweep incl. line + blind chains | Integration (ADR-050 job) | Identical primitive stream + `(start_pwm,end_pwm)`; no drift |
| TC8 | CTPS consumes recorded `(start_pwm,end_pwm)` for a blind run | Unit | Whole-run trapezoid rendered from recorded speeds; no re-derivation |
| TC9 | FAST + REAR-led final-hop shed vs rear overshoot window | Compile-time | `static_assert` fires iff shed > window (ADR-058 edge) |
| TC10 | Non-chain single hop (regression) | Unit/Integration | Byte-identical to pre-change: `resolve_effective_speed` default, per-hop profile, CTPS unchanged |

**Test data:** the existing `collinear_4node` fixture (line); a NEW collinear-blind fixture (`tools/path_planner/tests/fixtures/`); golden vectors dumped from a host `ChainSpeedPolicy` TU (the ADR-058 golden mechanism). **Run commands:** `uv run --extra dev pytest tools/path_planner/tests/`; `ctest --test-dir build/test --output-on-failure`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | ADR-059 + index | — | low | `docs/decisions/059-collinear-run-continuous-motion.md`, `DECISIONS.md` | Write ADR-059 amending ADR-023/046/052/058: blind-chain pass-through via backward-pass speed chaining; CTPS run-grouper truth-fix; keep 0.5° tolerance; guards. Update the DECISIONS.md index + status table. Verify no ADR-number collision first (memory `gotcha-adr-number-collision`). Satisfies the ADR requirement. |
| T2 | CTPS truth-fix (line/proj run-grouper) | — | high | `tools/path_planner/ctps/track_builder.py`, `tools/path_planner/ctps/profile.py` | Fix `_primitive_drive_speed` so pass tags are not forced `end=0`; add a run-grouper that merges consecutive pass-tagged primitives into ONE `DriveProfile` over the summed chord distance and slices `sample_drive` at each primitive's cumulative-distance boundary, preserving each primitive's snapshot-pose pin (TC7). Standalone; works off EXISTING line/proj pass tags. Satisfies AC1, TC2. |
| T3 | `ChainSpeedPolicy.h` + host golden TU | — | high | `WRO2026/src/drive/ChainSpeedPolicy.h`, `WRO2026/test/` (golden dump TU + CMake) | Header-only pure `resolveChainSpeeds` per the Contracts signature (backward pass, float32/int16 discipline, ADR-058 pattern). Add a host TU that dumps golden vectors. Satisfies AC3, AC4, TC1. |
| T4 | Python mirror `chain_speed.py` + golden parity | T3 | high | `tools/path_planner/ctps/chain_speed.py` (new), `tools/path_planner/tests/test_chain_speed.py` (new) | Pure float32 mirror of `resolveChainSpeeds` (numpy float32 + int16 truncation, like `ctps/profile.py`). Golden-vector parity test against T3's dump. Satisfies AC4, TC1. |
| T5 | Config knobs + static_asserts | — | med | `WRO2026/config/ConfigNav.h`, `WRO2026/config/ConfigDrive.h`, `WRO2026/config/ConfigChecks.h` | Add `PATH_BLIND_CHAIN_ENABLED` (default true) + `PATH_BLIND_CHAIN_MAX_CM` with per-constant `@warning` + tripwire (memory `project-config-constant-tripwire`); add the FAST+REAR final-hop-shed `static_assert` (AC8/TC9). Satisfies AC5(part), AC8. |
| T6 | Firmware PathTracker blind-chain chaining | T3, T5 | xHigh | `WRO2026/src/mission/PathTracker.cpp`, `WRO2026/src/mission/PathTracker.h` | Detect a collinear blind run (walk-ahead reusing `_passThroughHeadingOk` collinearity + same-direction; eligible = not pivot/seek/reset/goal, within cap, kill-switch on); call `ChainSpeedPolicy`; dispatch mid-hops `driveDist(...).tShape().setStartSpeed(vCarry).setEndSpeed(vNext).setBrake(false)`, final hop braked; stamp `onDeadReckonedAdvance` per node; fixed-size run buffer (`MAX_CHAIN_HOPS`). Guards per Error Handling. Satisfies AC2, AC3, AC5, TC3–TC6. |
| T7 | Sim mirror + parity fields | T4, T5, T6 | high | `tools/path_planner/sim/runtime.py`, `tools/path_planner/sim/parity.py`, `tools/path_planner/ctps/trace_loader.py` | Mirror blind-chain eligibility; emit `ACT_BLIND_PASS` (3-place contract) + additive `start_pwm`/`end_pwm` parity fields (consume `chain_speed.py`); loader accepts the fields (strict int-or-none) + the new tag. Regex-parity test for the two new Config knobs. Satisfies AC6, TC7. |
| T8 | CTPS consume chain speeds + blind-pass | T2, T7 | high | `tools/path_planner/ctps/track_builder.py`, `tools/path_planner/ctps/trace_loader.py`, `tools/path_planner/ctps/profile.py` | Extend the T2 run-grouper to include `ACT_BLIND_PASS` and to consume recorded `(start_pwm,end_pwm)` for blind runs (whole-run trapezoid from recorded speeds; never re-derive). Satisfies AC7, TC8. |
| T9 | Host C++ tests | T6 | med | `WRO2026/test/test_path_tracker_blind_chain.cpp` (new), `WRO2026/test/CMakeLists.txt` | ctest for the blind-chain FSM (TC3–TC6) + wire the `ChainSpeedPolicy` golden TU. Link real `PathTracker.cpp` per the host harness rules (memory `gotcha-host-tests-link-production-routes`, `project-host-test-link-toy-routes` if a chain graph the field lacks is needed). Satisfies AC2/AC5 verification. |
| T10 | Python/CTPS/parity tests + fixtures | T4, T8 | med | `tools/path_planner/tests/test_pass_through.py`, `tools/path_planner/tests/test_ctps_*.py`, `tools/path_planner/tests/fixtures/` | New collinear-blind fixture; CTPS run-grouper unit tests (line + blind, TC2/TC8/TC10); parity sweep coverage (TC7); regime/regression tests. Satisfies AC1, AC6, AC7, AC9. |

## Notes for Implementer

- **Do NOT touch `field_data.json` or `NAV_TURN_TOLERANCE_DEG`.** Diego owns field geometry and is straightening the CY columns in parallel. The feature is validated by fixtures, not by the CY1→CY3 path.
- **The ramp-down distance is `(cruise − PWM_MIN)/accel` (shed-to-floor), NOT `cruise/accel`.** The stop is `cruise→floor→brake` (ADR-058). Sizing anything against `cruise/accel` (17 cm) is the mistake that made the original mechanism look necessary.
- **No cross-hop state in `LineFollower`/`Drive`.** All run-scoping lives in `PathTracker`'s walk-ahead + `ChainSpeedPolicy` (pure). The controllers stay per-command. This is the ADR-050 boundary.
- **The inter-hop handoff holds the last PWM for ~1–2 ticks** (≈0.5–1 cm un-gated travel per node boundary at cruise). Bounded and non-accumulating (each hop's own encoder gate measures its own distance). The parity normalizer must treat a node stamped at `isIdle`-with-held-speed the same as at-rest for blind nodes (bookkeeping-only stamp).
- **CTPS grouper preserves pose boundaries** — only timing/velocity between snapshot poses changes, so the TC7 byte-pin holds and the frozen player-asset tests should not churn (verify; update once intentionally if they do — memory `ctps-timeline-lane-manifest-coupling`, `gotcha-byte-pin-test-crlf-lf`).
- **3-place tag contract** (`gotcha-ctps-drive-method-contract`) and **additive parity field discipline** (`gotcha-parity-record-kind-is-discriminator`) are load-bearing — add `ACT_BLIND_PASS` and the speed fields to ALL required places in the same change.
- **Golden/float discipline** for `ChainSpeedPolicy` mirror: numpy float32 + int16 truncation exactly like `ctps/profile.py` (`_f32`, `_trunc_i16`). Do not `//`-floor a signed divide.
- **Host build/test** is available locally (memory `host_cpp_toolchain_available`, `arduino_cli_headless_available`); run the FULL ctest + FULL pytest (contract changes ripple through fixtures — memory `gotcha-field-contract-change-breaks-cpp-and-py-fixtures`, `gotcha-strict-emitter-field-fixtures`).
- **Rollback:** `PATH_BLIND_CHAIN_ENABLED=false` restores today's stop-at-every-blind-node behavior; the CTPS grouper falls back to per-primitive rendering on any malformed run.
- **Sequencing note:** T2 is intentionally standalone (Fable's Approach C) — it can land + be eyeballed first to confirm line chains already cruise before the firmware blind work.
