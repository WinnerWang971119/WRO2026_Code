# Blind-column overshoot: braked-blind carry generalization + chain final-hop stub guard

**Goal:** Make a dead-reckoned collinear blind run (e.g. `goTo("MWU")` up the mosaic column) land its final node within <1 cm instead of driving ~8 cm too far into the top wall — without reopening ADR-059's rejected run-coalescing.

**Approach:** The per-hop *landing* is already accurate on this chassis (Diego, bench: a short single hop is as accurate as a long one). The overshoot is introduced only by hop *sequencing*, in two distinct defects: (1) with chaining **off**, the ~1 cm inter-hop boundary carry is not compensated on the plain per-hop braked path, so it accumulates over 8 hops ≈ 8 cm; (2) with chaining **on**, the measured-carry subtraction shrinks the final hop below `TSHAPE_MIN_MOVE_CM`, so the degraded stub drives it **flat at the plan start PWM (~67) and brakes from speed** into the wall. Fix both in place: generalize the existing ADR-059 measured-carry subtraction to *any* consecutive braked blind hops (companion "D"), and guard the chained final-hop degraded stub so it crawls at the floor instead of braking from the plan PWM. Re-enable blind chaining and revert the three committed diagnostics. If bench shows this still exceeds <1 cm, fall back to the documented contingency **Option A** (one combined braked `driveDist` per collinear blind run).

## Scope

- **In scope:**
  - Generalize the measured inter-hop boundary-carry subtraction (today gated inside `if (_chain.active)`) to *all consecutive braked blind dead-reckon hops*, including the chaining-disabled / chain-ineligible per-hop path — with an explicit contiguity/reset lifecycle so a stale carry is never applied.
  - Guard the ADR-059 chained-run degraded stub (`chainHop && dispatchCm < TSHAPE_MIN_MOVE_CM`) so a **final** hop dispatches flat at the motor floor (`±PWM_MIN_MAGNITUDE`), not at the plan start PWM.
  - Re-enable blind chaining (`PATH_BLIND_CHAIN_ENABLED_DEFAULT` → `true`).
  - Revert the three committed diagnostic changes (see Decisions): restore `testOne`, remove the `[HOP]` bench-instrumentation block, flip the chain flag.
  - A host ctest asserting the **real dispatched Drive setpoint** for (a) the stub-guard (final degraded hop launches at floor, not plan PWM) and (b) the generalized non-chain carry (a seeded carry shrinks the next braked blind command), because the parity/sim/CTPS traces cannot observe either.
  - A new ADR (ADR-079) amending ADR-059: carry generalized beyond chains; final-hop degraded-stub floor pin; coalescing remains rejected but is recorded as the bench fallback.
- **Out of scope:**
  - Run-coalescing / one combined move (Option A) — documented as the **fallback contingency only**, implemented in a follow-up plan *only if* this fix fails bench verification (Diego, 2026-07-22).
  - A `ChainSpeedPolicy` terminal brake-to-rest reserve (Fable's "Option 2-sharpened") — dropped because short hops brake accurately on this chassis, so the reserve solves a non-problem here. (Its rationale is preserved in Notes in case bench data flips the premise.)
  - LINE / PROJECTION collinear pass-through (they re-seek the line at each node and self-correct; the bug is blind-only).
  - Re-tuning any drive constant (`TSHAPE_ACCEL_PWM_PER_CM`, `PWM_MIN_MAGNITUDE`, `SPEED_NORMAL`) — no constant changes, so the drive-constant golden fan-out does not apply.
  - `MISSION_ALLOW_REARM_DEFAULT` (intentionally `true` on bench — leave it).

## Decisions

- **Fix = carry-generalization (D) + stub-guard, not coalescing (Option 1) and not a terminal reserve.** — Diego's bench fact (short single hops land accurately) removes coalescing's only unique advantage (an accurate combined landing) and removes the reserve's rationale (short-hop braking is fine). What remains wrong is purely the *sequencing* carry and the *chained-final-stub* dispatch, both fixable in place. This also avoids ADR-059's explicit rejection of coalescing on parity-granularity grounds and its four-surface fan-out.
- **Option A (combined move) is the documented fallback, not the primary.** — If D + stub-guard still exceeds <1 cm at MWU on the bench, escalate to a follow-up plan implementing one braked `driveDist(total)` per strictly-collinear blind run (accepting the parity-granularity trade recorded in ADR-059, which its own revisit-trigger anticipates). Diego, 2026-07-22.
- **Working mechanism hypothesis (bench is the arbiter):** the `[HOP]`-logged ~1 cm/hop excess is the inter-hop boundary carry (chassis rolling between the previous brake and the next hop's encoder baseline), which a single isolated hop never incurs — so generalizing the carry subtraction compensates exactly it. If bench instead shows a per-hop *bias* in the blind `goTo` path that a raw `driveDist` lacks, the fix target changes and this plan is revisited (see Notes → Bench gate).
- **The three "reverts" are new edits, not a `git checkout`.** — They were committed in `a22cc9f "Test pathtracker"`, so `git diff` is clean. Restoring them is source editing: (1) `ConfigNav.h:709` `PATH_BLIND_CHAIN_ENABLED_DEFAULT false` → `true`; (2) remove the `[HOP]` `Serial` block at `PathTracker.cpp:3355-3373`; (3) restore the original `testOne` body from the comment at `Tasks.cpp:341-362` and drop the diagnostic body (`holdYaw()` + `driveDist(100)`) + the commented instrumented climb.
- **No sim/parity/CTPS mirror change.** — D changes behavior only when the measured carry > 0; the parity harness and sim park encoders (carry = 0), so the emitted trace (`dist_cm`, `start_pwm`, `end_pwm`, `chain`) is byte-identical. The stub-guard changes only the dispatched PWM, which the parity emit deliberately does not record (it emits the plan endpoints). Both are therefore invisible to the trace and MUST be pinned by a host ctest on the real setpoint instead.
- **New ADR-079 amends ADR-059** (not a supersession): the two-pass `ChainSpeedPolicy` and hop-granular parity survive unchanged; ADR-079 extends the carry model beyond chains and fixes the final-hop degraded-stub dispatch.

## Acceptance Criteria

- [ ] AC1 — With chaining **on**, `goTo("MWU")` from `i2` up the 8-hop blind column (~70 cm) lands the chassis within **<1 cm** of MWU on the bench (ruler), clear of the top wall. *(Bench-verified; not a host test.)*
- [ ] AC2 — With chaining **off**, the same 8-node climb (per-node goTos) no longer accumulates: final node lands within <1 cm; no monotonic per-hop creep. *(Bench-verified.)*
- [ ] AC3 — Host ctest: a chained **final** hop forced below `TSHAPE_MIN_MOVE_CM` by a seeded boundary carry dispatches its degraded stub at the **floor** (`±PWM_MIN_MAGNITUDE`), asserted against the real `MiniR4.DriveDC` mock setpoint (`lastMoveSyncL/R`) — NOT the plan start PWM, and still brakes (`lastBrake == true`).
- [ ] AC4 — Host ctest: with chaining **disabled**, a sequence of ≥3 consecutive braked blind hops each seeded with a boundary carry has the carry subtracted from each subsequent commanded distance (verified via the same proxy TC8 uses: arrival step-count bound + exact final-node landing), so no drift accumulates over N hops.
- [ ] AC5 — Host ctest: the generalized carry state **resets** and does not apply a stale carry across a discontinuity — a turn, a non-blind hop, a STOP_AT_LINE/seek arrival, a fault, or mission idle between two braked blind hops leaves the following hop's command un-adjusted (full distance).
- [ ] AC6 — `PATH_BLIND_CHAIN_ENABLED_DEFAULT` is `true`; `testOne` is restored to its original 8-node climb; the `[HOP]` block is gone. Confirmed by reading the three files.
- [ ] AC7 — Full host `ctest` (all targets) and full `uv run --extra dev pytest tools/path_planner/tests/` pass unchanged; the PathTracker parity job stays green with **no** schema or golden edits (proving the fix is trace-invisible).
- [ ] AC8 — `routes.h/.cpp` unchanged (no field/emitter edit); CI routes-freshness stays green.
- [ ] AC9 — ADR-079 written, DECISIONS.md index + status table updated, ADR-059 cross-linked ("amended by ADR-079").

## Defect Analysis (root cause, verified in code)

1. **Chaining-off accumulation.** The measured-carry subtraction lives only inside `if (_chain.active)` (`PathTracker.cpp:3237-3306`). The plain per-hop braked dispatch (`else` branches at `3466-3475` GYRO / `3524-3532` SYNC) has **no** carry compensation. Each braked stop leaves the chassis ~1 cm past the node; `Localization.onDeadReckonedAdvance()` (`PathTracker.cpp:1279`) re-anchors belief to the exact node and discards it; the next hop drives its full geometric distance from a point already ~1 cm past → accumulates ≈ 8 cm over the column.

2. **Chaining-on final-hop stub.** The chain backward pass (`ChainSpeedPolicy.h:212-229`) plans the final hop's entry PWM (≈67 for the 6.4 cm hop) against the **full** geometric length. Dispatch then subtracts the measured carry from the **distance** (`PathTracker.cpp:3271`, `dispatchCm = effectiveDistCm - carryCm`), dropping it below `TSHAPE_MIN_MOVE_CM` (5 cm). The degraded-stub branch (`3375-3409`) then dispatches `stubPwm = chainMid ? chainEndPwm : signedStartSpeed` — for a **final** hop, `signedStartSpeed` ≈ 67 — as a `.flat().setBrake(true)` move: it drives the shrunk hop flat at ~67 PWM and brakes from speed → the wall hit.

## Contracts & Interfaces

Single source of truth for the cross-task seams. Only `PathTracker` internals change; the wire/trace contract is deliberately unchanged.

### New PathTracker state (T-D) — generalized braked-blind carry

Add persistent members to `PathTrackerClass` (owner: **T-D**; consumers: **T-STUB** reads none of these, **T-TEST**):

```cpp
// PathTracker.h — private members, near the _chain (ChainRun) block (~1241-1254)
bool    _brakedBlindCarryActive = false;  ///< a prior braked blind hop is a valid carry baseline
int32_t _brakedBlindEncSum      = 0;      ///< encoder sum snapshot at that hop's dispatch
float   _brakedBlindCmdCm        = 0.0f;  ///< distance commanded at that hop's dispatch
```

- **Snapshot rule:** on dispatching *any* braked blind hop (chain-final OR non-chain single), set `_brakedBlindCarryActive = true`, `_brakedBlindEncSum = Drive.getEncL()+getEncR()`, `_brakedBlindCmdCm = dispatchCm`. (A mid-chain non-braking hop is handled by the existing `_chain.dispatch*` baselines; do not disturb those.)
- **Read rule:** when about to dispatch a braked blind hop and `_brakedBlindCarryActive`, compute `carry = |encDegToCm((encSum - _brakedBlindEncSum)/2)| - _brakedBlindCmdCm`, clamp `< 0 → 0`, subtract from `dispatchCm`, floor to `PIVOT_TRIM_MIN_DRIVE_CM` — identical arithmetic to the existing AC9 block (`3263-3275`), reused so the two paths cannot drift. When the chain path is already active, its own `_chain.dispatch*` baselines take precedence; the generalized state services only the non-chain path (do not double-subtract).
- **Reset rule (invalidate `_brakedBlindCarryActive = false`):** any pre-edge turn dispatch, any non-blind hop dispatch (LINE / PROJECTION / seek / STOP_AT_LINE), any wall/line re-anchor, any fault latch, `PathTracker` returning to `IDLE`, and a fresh `goTo` / `_onPlanNextHop` origin bind. (Prefer routing the invalidation through the same points that already reset run state / `_chainReset()`.)

### Stub-guard (T-STUB) — dispatch-only

At `PathTracker.cpp:3386`, change the final-hop stub PWM from the plan start speed to the floor:

```cpp
// was: const int16_t stubPwm = chainMid ? chainEndPwm : signedStartSpeed;
const int16_t floorPwm = decision.reverse ? (int16_t)-Config::PWM_MIN_MAGNITUDE
                                           : (int16_t) Config::PWM_MIN_MAGNITUDE;
const int16_t stubPwm  = chainMid ? chainEndPwm : floorPwm;   // final hop crawls at floor, not plan PWM
```

- A mid-chain stub keeps flowing at its planned exit (`chainEndPwm`); only the **final** (braked) stub changes. This is a dispatch detail — the `#ifdef PT_PARITY_TRACE` emit (`3548-3556`) still records `signedStartSpeed`/`0`, so the trace is unchanged (mirrors the existing `chainFinalEndPwm` floor pin).

### Unchanged contracts (must NOT drift)

- Parity schema (`sim/parity.py` `blind_dead_reckon` = `{node, mode, dist_cm, start_pwm, end_pwm, chain, reanchor}`, `blind_pass = {node}`) and `PathTrackerParityTrace.h` — **no field add/change**.
- `ChainSpeedPolicy.h` / `chain_speed.py` / `chain_speed_goldens.json` — **untouched** (no reserve).
- CTPS `track_builder.py` run-grouper — **untouched**.
- `routes.h/.cpp`, `field_data.*` — **untouched**.

## Testing Strategy

**Levels:** Host C++ ctest (dispatch-setpoint truth), Python parity/pytest (unchanged-trace proof), Bench (the real acceptance).

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | Chained final hop forced below `TSHAPE_MIN_MOVE_CM` via seeded carry | Host ctest | Dispatched `lastMoveSyncL/R == ±PWM_MIN_MAGNITUDE` (floor), NOT plan start (~67); `lastBrake == true` (AC3) |
| TC2 | Chaining OFF, ≥3 consecutive braked blind hops each with a seeded carry (`driveChainHopExact`-style) | Host ctest | Each next command reduced by the measured carry; final node lands on-node; arrival step-count bounded (AC4) |
| TC3 | Carry state reset across a turn between two braked blind hops | Host ctest | Second hop commands the FULL distance (no stale carry) (AC5) |
| TC4 | Carry state reset across a non-blind (LINE/seek) hop, and across a fault→clear | Host ctest | Following braked blind hop commands full distance (AC5) |
| TC5 | Existing chain carry (TC8 in `test_path_tracker_blind_chain.cpp`) still passes | Host ctest | No regression to the in-chain carry path |
| TC6 | Full parity trace diff (firmware host vs sim), goldens, routes-freshness | pytest/ctest | All green with zero schema/golden edits (AC7, AC8) |
| TC7 | Bench: `goTo("MWU")` chaining ON, and the 8-node climb chaining OFF | Bench | Land <1 cm from MWU, clear of wall (AC1, AC2) — the gate that decides fix-vs-Option-A |

**Test data:** reuse the `MiniR4.DriveDC` mock (`lastMoveSyncL/R`, `lastBrake`, `brakeCallCount`) and the `driveChainHopExact(commandedCm, carryCm)` carry-seeding helper already in `test_path_tracker_blind_chain.cpp`. New cases go in that file (or a sibling `test_path_tracker_blind_carry.cpp` if it grows) with a matching CMake target + `add_fixture_routes_force_include` + `add_path_force_includes`.
**Run commands:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`; `uv run --extra dev pytest tools/path_planner/tests/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Re-enable blind chaining | — | low | `WRO2026/config/ConfigNav.h` | Flip `PATH_BLIND_CHAIN_ENABLED_DEFAULT` `false`→`true` (line 709); drop any temp comment. Satisfies AC6. |
| T2 | Restore `testOne` | — | low | `WRO2026/Tasks.cpp` | Replace the diagnostic body (`holdYaw()`+`driveDist(100)` at 338-339) and the commented instrumented climb (312-337) with the original 8-node climb preserved at 341-362; remove the now-consumed comment scaffolding. Satisfies AC6. |
| T3 | Remove `[HOP]` instrumentation | — | low | `WRO2026/src/mission/PathTracker.cpp` | Delete the `// TEMP BENCH INSTRUMENTATION` `Serial` block (3355-3373). Same-file with T4/T5 → runs first in the PathTracker chain. Satisfies AC6. |
| T4 | Stub-guard: final degraded hop brakes from floor | T3 | med | `WRO2026/src/mission/PathTracker.cpp` | Per Contracts §Stub-guard: final-hop `stubPwm` → `±PWM_MIN_MAGNITUDE` (reverse-signed), mid-hop unchanged. Parity-invisible → its correctness is only provable by TC1. Satisfies AC3. |
| T5 | Generalize measured carry to braked blind sequences (companion D) | T4 | high | `WRO2026/src/mission/PathTracker.cpp`, `WRO2026/src/mission/PathTracker.h` | Per Contracts §New state: add the 3 members; snapshot on every braked blind dispatch; read+subtract on a contiguous braked blind hop reusing the exact AC9 arithmetic; wire the reset rule through existing run-reset/turn/non-blind/fault/idle points; do not double-subtract while `_chain.active`. Satisfies AC4, AC5. |
| T6 | Host ctests for stub-guard + generalized carry + resets | T4, T5 | med | `WRO2026/test/test_path_tracker_blind_chain.cpp` (or new sibling), `WRO2026/test/CMakeLists.txt` | Implement TC1–TC5 asserting the **real** `MiniR4.DriveDC` setpoint (not the plan). If a new file, add the CMake target + force-includes. Satisfies AC3-AC5. |
| T7 | Regression + trace-invariance verification | T5, T6 | low | (run-only) | Full `ctest` + full `uv run --extra dev pytest`; confirm parity job green with zero schema/golden/routes edits; confirm `git status` shows no `routes.*`/goldens/parity changes. Satisfies AC7, AC8. |
| T8 | ADR-079 amending ADR-059 | T4, T5 | low | `docs/decisions/079-braked-blind-carry-and-stub-guard.md`, `DECISIONS.md`, `docs/decisions/059-collinear-run-continuous-motion.md` | New ADR: carry generalized beyond chains; final-hop degraded-stub floor pin; coalescing stays rejected but is the recorded bench fallback (update ADR-059's revisit note + add "amended by ADR-079"); bump DECISIONS.md index + status count. Satisfies AC9. |

Parallelism: T1, T2, T8 run parallel to the PathTracker chain. T3→T4→T5 are serialized (same file). T6 waits for T4+T5; T7 waits for T5+T6. (T8 may start after T4/T5's design is settled; it only documents, no code dependency beyond the decisions.)

## Notes for Implementer

- **Bench gate = the real acceptance.** AC1/AC2/AC7 (bench) decide success. Host tests prove the dispatch is *what we intend*; only the ruler at MWU proves it *fixes the overshoot*. If the bench still exceeds <1 cm after D + stub-guard, STOP and escalate to the Option A follow-up plan — do not keep tuning. Also re-check the mechanism hypothesis: if the residual looks like a per-hop *bias* (constant offset independent of hop count) rather than an accumulating carry, the blind `goTo` path may add a per-hop trim/offset a raw `driveDist` lacks — investigate before Option A.
- **Reuse, don't re-derive, the carry arithmetic.** T5 must call the *same* `traveledCm`/`carryCm` computation the in-chain AC9 block uses (`3263-3270`), ideally by extracting a small private helper both paths call, so they can never drift (`gotcha-drive-constant-retune-fanout` family — one shared artifact).
- **Do not double-subtract.** While `_chain.active`, the chain's own `_chain.dispatch*` baselines own the carry; the generalized `_brakedBlindCarry*` state services only the non-chain braked path. Guard the read so a hop never subtracts both.
- **Parity-invisible ⇒ test the setpoint.** Per `gotcha-parity-ctps-record-plan-not-dispatch`: the stub-guard and the carry both vanish from the parity/sim/CTPS trace (parked encoders, plan-not-dispatch emit). TC1/TC2 MUST assert `lastMoveSyncL/R` (and `lastBrake`), not the recorded plan — a green parity job is NOT evidence either works.
- **RAII builder rule.** Any Drive builder edit stays one discarded-temporary `.async()` statement (`gotcha_drive_builder_raii`); don't bind a builder to a local.
- **`SYNC` is broken on this chassis** (crossed encoder cables); the column runs `GYRO`. The stub-guard touches both the SYNC and GYRO branches' shared `stubPwm`, so it is mode-agnostic — verify both branches (3390-3408) use the guarded value.
- **Dropped-but-recorded alternative (terminal reserve).** If bench data later shows short-hop braking is *not* accurate after all (contradicting the current premise), the sharper fix is Fable's `ChainSpeedPolicy` terminal reserve: plan the floor to arrive `R ≈ 4 cm` before the run end (`terminal entry = floor + accel·max(0, d_final − R)`) so the shed fits after the carry subtraction. It lives inside the golden-pinned policy + `chain_speed.py` mirror (re-pin goldens) with parity shape unchanged. Kept here as a second fallback, not implemented now.
- **Approach provenance.** Fable (approach consult) **vetoed** the original Option 1 (coalescing) citing ADR-059's explicit prior rejection + the verified carry-blind/stub root cause, and recommended the carry-generalization + stub fix that became this plan (its terminal-reserve half dropped on Diego's short-hop-accuracy bench fact). Codex consult unavailable (forwarder returned an unretrievable background task handle).
