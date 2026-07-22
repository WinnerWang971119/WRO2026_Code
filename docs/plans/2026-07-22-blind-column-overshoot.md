# Blind-column overshoot: braked-blind carry generalization + chain final-hop stub guard

**Goal:** Make a dead-reckoned collinear blind run (e.g. `goTo("MWU")` up the mosaic column) land its final node within <1 cm instead of driving ~8 cm too far into the top wall — without reopening ADR-059's rejected run-coalescing.

**Approach:** The per-hop *landing* is already accurate on this chassis (Diego, bench: a short single hop is as accurate as a long one). The overshoot is introduced only by hop *sequencing*, in two distinct defects: (1) with chaining **off**, the boundary carry (the encoder overshoot each braked stop leaves, which `onDeadReckonedAdvance` then discards by re-anchoring to the exact node) is not compensated on the plain per-hop braked path, so it accumulates over 8 hops ≈ 8 cm; (2) with chaining **on**, the measured-carry subtraction shrinks the final hop below `TSHAPE_MIN_MOVE_CM`, so the degraded stub drives it **flat at the plan start PWM (~67) and brakes from speed** into the wall. Fix both in place: generalize the existing ADR-059 measured-carry subtraction to *any* consecutive braked blind hops (companion "D"), and guard the chained final-hop degraded stub so it crawls at the floor instead of braking from the plan PWM. Re-enable blind chaining and revert the three committed diagnostics. If bench shows this still exceeds <1 cm, fall back to the documented contingencies (terminal reserve first, then Option A — see Decisions).

## Scope

- **In scope:**
  - Generalize the measured inter-hop boundary-carry subtraction (today gated inside `if (_chain.active)`) to *all consecutive braked blind dead-reckon hops*, including the chaining-disabled / chain-ineligible per-hop path — with a precise, chain-lifecycle-independent state machine (see Contracts) so a stale, cross-direction, or double-counted carry is never applied.
  - Guard the ADR-059 chained-run degraded stub (`chainHop && dispatchCm < TSHAPE_MIN_MOVE_CM`) so a **final** hop dispatches flat at the motor floor (`±PWM_MIN_MAGNITUDE`), not at the plan start PWM.
  - Re-enable blind chaining (`PATH_BLIND_CHAIN_ENABLED_DEFAULT` → `true`).
  - Revert the three committed diagnostic changes (see Decisions): restore `testOne`, remove the `[HOP]` bench-instrumentation block, flip the chain flag.
  - Host ctests asserting the **real dispatched Drive setpoint** for (a) the stub-guard, (b) the generalized non-chain carry + its resets, and (c) the chain↔non-chain handoff and divergence cases — because the parity/sim/CTPS traces cannot observe any of them.
  - A new ADR (ADR-079) amending ADR-059: carry generalized beyond chains; final-hop degraded-stub floor pin; coalescing remains rejected but is recorded as a fallback.
- **Out of scope:**
  - Run-coalescing / one combined move (**Option A**) — documented as the **second fallback contingency**, implemented in a follow-up plan only if the fix *and* the terminal reserve fail bench (Diego, 2026-07-22).
  - A `ChainSpeedPolicy` terminal brake-to-rest reserve — **first fallback**, not primary (see Decisions W4 note); dropped from the primary because short *profiled* hops brake accurately on this chassis.
  - LINE / PROJECTION collinear pass-through (they re-seek the line at each node and self-correct; the bug is blind-only).
  - Re-tuning any drive constant. **Exception (already committed, kept):** `GYRO_HOLD_KP`/`KD` were changed in `a22cc9f` and are an intentional bench tune Diego is keeping (Decisions) — this plan does NOT touch them and does NOT trigger the drive-constant golden fan-out (they are not `ChainSpeedPolicy`/`TShapeProfile` inputs).
  - `MISSION_ALLOW_REARM_DEFAULT` (intentionally `true` on bench — leave it).

## Decisions

- **Fix = carry-generalization (D) + stub-guard, not coalescing (Option 1) and not a terminal reserve.** — Diego's bench fact (short single hops land accurately) removes coalescing's only unique advantage (an accurate combined landing) and removes the reserve's rationale (short *profiled*-hop braking is fine). What remains wrong is the *sequencing* carry and the *chained-final-stub* dispatch, both fixable in place. This also avoids ADR-059's explicit rejection of coalescing on parity-granularity grounds and its four-surface fan-out.
- **Fallback ordering (bench-gated):** if D + stub-guard still exceeds <1 cm at MWU: **(1st) terminal reserve** — plan the chain final hop to reach the floor `R ≈ 4 cm` before the run end so the post-carry residual stays ≥ `TSHAPE_MIN_MOVE_CM` and takes the *profiled* braked path (the regime Diego's bench certifies as accurate); this lives inside the golden-pinned `ChainSpeedPolicy.h` + `chain_speed.py` mirror (re-pin goldens), parity shape unchanged. **(2nd) Option A** — one braked `driveDist(total)` per strictly-collinear blind run (accepts ADR-059's parity-granularity trade). Diego, 2026-07-22.
- **W4 momentum-residual risk (accepted, Fable):** the stub-guard commands a flat floor (45) PWM, but the chassis *enters* the shrunk final hop at the previous mid-hop's exit speed (~67-equivalent); a flat command sheds that only through passive drag, so the brake may still fire above floor speed and a smaller coast persists. The stub-guard is therefore lower-confidence than the carry generalization. **Failure-signature routing:** if the bench shows AC1 (chaining-on final node) failing while AC2 (chaining-off accumulation) passes, that isolates the stub, so escalate to the **terminal reserve** (1st fallback), *not* Option A.
- **GYRO_HOLD gains are an intentional bench tune, kept.** — `a22cc9f` raised `GYRO_HOLD_KP` 1.6→3.0 and `GYRO_HOLD_KD` 0.051→0.07 (ADR-075 `GyroHoldPolicy` straight-line heading-hold PD). Diego confirms this is deliberate (2026-07-22); it is NOT reverted here and the bench acceptance runs under it. It affects heading-hold, not commanded distance, so it is not implicated in the overshoot.
- **Option A (combined move) stays rejected as primary.** — See Fallback ordering above; Diego, 2026-07-22.
- **Working mechanism hypothesis (bench is the arbiter):** the `[HOP]`-logged ~1 cm/hop excess is the inter-hop boundary carry (chassis rolling holding the last PWM across the node boundary before the next hop's encoder baseline), which a single isolated hop never incurs — so generalizing the carry subtraction compensates exactly it. The *code* verifies the missing compensation; it does not prove the physical mechanism. If bench shows a per-hop *bias* in the blind `goTo` path that a raw `driveDist` lacks (a constant offset independent of hop count, not an accumulating carry), the fix target changes and this plan is revisited (Notes → Bench gate).
- **The three "reverts" are new edits, not a `git checkout`.** — They were committed in `a22cc9f "Test pathtracker"`, so `git diff` is clean. Restoring them is source editing: (1) `ConfigNav.h:709` `PATH_BLIND_CHAIN_ENABLED_DEFAULT false` → `true`; (2) remove the `[HOP]` `Serial` block at `PathTracker.cpp:3355-3373`; (3) restore the original `testOne` body from the comment at `Tasks.cpp:341-362` and drop the diagnostic body (`holdYaw()` + `driveDist(100)` at 338-339) + the commented instrumented climb (`Tasks.cpp:320-337`; 312-319 is the banner).
- **No sim/parity/CTPS mirror change.** — D changes behavior only when the measured carry > 0; the parity harness and sim park encoders (carry = 0), so the emitted trace (`dist_cm`, `start_pwm`, `end_pwm`, `chain`) is byte-identical. The stub-guard changes only the dispatched PWM, which the parity emit deliberately does not record (it emits the plan endpoints). Both are therefore invisible to the trace and MUST be pinned by host ctests on the real setpoint instead.
- **Branch is currently red.** — `a22cc9f` flipped `ConfigNav.h:709` to `false` but not the sim mirror `runtime.py:376` (`= True`), so `test_adr059_path_blind_chain_enabled_parity` (`tools/path_planner/tests/test_pass_through.py`) currently FAILS on this branch. T1 (flip the header back to `true`) heals it; no sim edit is needed (the mirror is already `True`).
- **New ADR-079 amends ADR-059** (not a supersession): the two-pass `ChainSpeedPolicy` and hop-granular parity survive unchanged; ADR-079 extends the carry model beyond chains and fixes the final-hop degraded-stub dispatch.

## Acceptance Criteria

- [ ] AC1 — With chaining **on**, `goTo("MWU")` from `i2` up the 8-hop blind column (~70 cm) lands the chassis within **<1 cm** of MWU on the bench (ruler), clear of the top wall. *(Bench-verified. If this fails while AC2 passes → terminal-reserve fallback, per Decisions.)*
- [ ] AC2 — With chaining compiled **off**, a single `goTo("MWU")` walking the 8-node climb as 8 braked blind hops within one uninterrupted dispatch (the FSM cycles BLIND_FORWARD→PLAN_NEXT_HOP→dispatch and never returns to IDLE between hops) no longer accumulates: final node lands within <1 cm; no monotonic per-hop creep. *(Bench-verified, matching host test TC2.)* Separate per-node goTos are NOT in scope for the carry — each passes through IDLE + a fresh-goTo origin bind, which resets the carry by design (AC5).
- [ ] AC3 — Host ctest: a chained **final** hop forced below `TSHAPE_MIN_MOVE_CM` by a seeded boundary carry dispatches its degraded stub at the **floor** (`±PWM_MIN_MAGNITUDE`), asserted against the real `MiniR4.DriveDC` mock setpoint (`lastMoveSyncL/R`) — NOT the plan start PWM, and still brakes (`lastBrake == true`). The **mid-hop** stub arm still dispatches `chainEndPwm` (assert both arms).
- [ ] AC4 — Host ctest: with chaining **off**, a single-goTo sequence of ≥3 consecutive braked blind hops each seeded with a boundary carry has the carry subtracted from each subsequent commanded distance (verified via the proxy TC8 uses: arrival step-count bound + exact final-node landing), so no drift accumulates over N hops.
- [ ] AC5 — Host ctest: the generalized carry state resets / does not apply a stale, cross-direction, or double-counted carry. Each of these leaves the following braked blind hop's command **un-adjusted (full distance)**: a turn, a non-blind hop, a STOP_AT_LINE/seek arrival, a wall/line re-anchor, a fault, a fresh-goTo origin bind (the IDLE→PLAN_NEXT_HOP transition of a new goTo), a **direction flip** (REAR after FRONT with no turn), a **RAW_FLAT / mid-chain non-braked** dispatch, and the **chain-divergence fallback** (3288-3305). "Mission idle between two braked blind hops" means the FSM actually returned to IDLE between them (separate goTos); within a single multi-hop goTo the FSM never reaches IDLE, so the carry persists across all of that goTo's hops (what AC2 exercises) — not a violation. Positive handoff: a chain-final braked hop → a following non-chain braked blind hop telescopes correctly (no double-subtract).
- [ ] AC6 — `PATH_BLIND_CHAIN_ENABLED_DEFAULT` is `true`; `testOne` is restored to its original 8-node climb; the `[HOP]` block is gone. Confirmed by reading the three files.
- [ ] AC7 — Full host `ctest` (all targets) and full `uv run --extra dev pytest tools/path_planner/tests/` pass; the PathTracker parity job stays green with **no** schema or golden edits (proving the fix is trace-invisible). Note the parity pytest is currently red on this branch and is healed by T1 (Decisions).
- [ ] AC8 — `routes.h/.cpp` unchanged (no field/emitter edit); CI routes-freshness stays green.
- [ ] AC9 — ADR-079 written, DECISIONS.md index + status table updated, ADR-059 cross-linked ("amended by ADR-079").

## Defect Analysis (root-cause STRUCTURE verified in code; physical mechanism bench-gated)

The code below verifies *where compensation is missing / mis-dispatched*; the physical "~1 cm" magnitude and its inter-hop-carry mechanism are the working hypothesis (Decisions), confirmed only by AC1/AC2 on the bench. The per-hop landing itself is accurate (Diego), so these are sequencing defects, not a short-hop braking defect.

1. **Chaining-off accumulation.** The measured-carry subtraction lives only inside `if (_chain.active)` (`PathTracker.cpp:3237-3306`, subtraction at `3266-3275`). The plain per-hop braked dispatch (`else` branches at `3466-3475` GYRO / `3524-3532` SYNC) commands `dispatchCm == effectiveDistCm` with **no** carry compensation. Whatever small boundary carry each braked stop leaves is discarded when `Localization.onDeadReckonedAdvance()` (`PathTracker.cpp:1279`) re-anchors belief to the exact node; the next hop then drives its full geometric distance from a physically-ahead start, and over the 8-hop column the uncompensated carry accumulates ≈ 8 cm.

2. **Chaining-on final-hop stub.** The chain backward pass (`ChainSpeedPolicy.h:212-229`) plans the final hop's entry PWM (≈67 for the 6.4 cm hop) against the **full** geometric length. Dispatch then subtracts the measured carry from the **distance** (`PathTracker.cpp:3271`), dropping it below `TSHAPE_MIN_MOVE_CM` (5 cm). The degraded-stub branch (`3375-3409`) then dispatches `stubPwm = chainMid ? chainEndPwm : signedStartSpeed` — for a **final** hop, `signedStartSpeed` ≈ 67 — as a `.flat().setBrake(true)` move: it drives the shrunk hop flat at ~67 PWM and brakes from speed. (Per Decisions W4, the stub-guard's flat-floor command may only *reduce*, not eliminate, this — hence the terminal-reserve fallback.)

## Contracts & Interfaces

Single source of truth for the cross-task seams. Only `PathTracker` internals change; the wire/trace contract is deliberately unchanged.

### New PathTracker state (owner: **T5**; consumers: **T4** reads none of these, **T6** tests)

Add persistent members to `PathTrackerClass` (`PathTracker.h`, near the `_chain (ChainRun)` block ~1241-1254):

```cpp
bool    _brakedBlindCarryActive = false;  ///< a prior braked blind hop is a valid carry baseline
int32_t _brakedBlindEncSum      = 0;      ///< encoder sum snapshot at that hop's dispatch
float   _brakedBlindCmdCm       = 0.0f;   ///< distance commanded at that hop's dispatch
bool    _brakedBlindReverse     = false;  ///< resolved direction of that hop (front/rear)
```

The generalized carry state is **INDEPENDENT of the `_chain` lifecycle**; `_chainReset()` MUST NOT touch these members. Precise state machine (owner T5 implements exactly this):

- **SNAPSHOT** — after *every* braked blind dispatch (chain-entry, chain-final, AND non-chain), unconditionally set `_brakedBlindEncSum = Drive.getEncL()+getEncR()`, `_brakedBlindCmdCm = dispatchCm`, `_brakedBlindReverse = decision.reverse`, `_brakedBlindCarryActive = true`. This runs even on an in-`_chain.active` hop where the chain owns the read, so a post-chain braked hop can telescope from the chain-final hop.
- **READ + SUBTRACT** — before dispatching a braked blind hop, subtract the carry only when ALL hold: `_brakedBlindCarryActive` AND the prior dispatch was a contiguous braked blind hop AND `_brakedBlindReverse == decision.reverse` AND NOT `match` (the in-`_chain.active` AC9 block is already subtracting its own carry on this hop — the `!match` guard is the sole double-subtract interlock). Carry arithmetic is the **same shared helper** the in-chain AC9 block uses: `carry = |encDegToCm((encSum - _brakedBlindEncSum)/2)| - _brakedBlindCmdCm`, clamp `<0→0`, `dispatchCm -= carry`, floor to `PIVOT_TRIM_MIN_DRIVE_CM`.
- **DIRECTION-FLIP** — when `_brakedBlindCarryActive` AND `_brakedBlindReverse != decision.reverse`: use carry = 0 for this hop (do NOT subtract, do NOT sign-flip/lengthen) and **re-arm** for the new direction (set `_brakedBlindReverse = decision.reverse`, `_brakedBlindEncSum = current encSum`, `_brakedBlindCmdCm = dispatchCm`) so the flipped hop begins a fresh telescoping chain. Reset-on-flip (not sign-aware add) is intentional: a forward overshoot is structurally near-cancelled by the reverse hop's own braking overshoot, leaving ≤ one carry of single-hop error that the next re-anchor zeroes; a sign-aware add reintroduces a 2×carry failure mode for negligible gain.
- **INVALIDATE** (`_brakedBlindCarryActive = false`) — on any NON-braked dispatch (mid-chain flow-through, RAW_FLAT); on the **chain-divergence fallback** (`3288-3305`) as an explicit step at the divergence site, NOT via `_chainReset()`, ordered BEFORE the same-invocation non-chain braked read at ≥`3326`; and on a turn dispatch, a non-blind hop (LINE/PROJECTION/seek/STOP_AT_LINE), a wall/line re-anchor, a fault latch, `PathTracker` returning to IDLE, and a fresh-goTo **origin bind**.
- **ORIGIN-BIND vs CONTINUATION** — the reset fires only on a fresh goTo's origin bind, defined as the first PLAN_NEXT_HOP entered on the **IDLE→PLAN_NEXT_HOP** transition of a new goTo. The per-hop `_onPlanNextHop` **continuation** re-entries (BLIND_FORWARD→PLAN_NEXT_HOP while the same goTo is active) must NOT reset. Distinguish the two by a **first-hop-of-goTo flag set on leaving IDLE**, never by the `_onPlanNextHop` call itself (which fires once per hop).
- **CHAIN-RETIRE ORDERING PIN** (`3565-3572`) — the chain-final braked dispatch snapshots the generalized baseline, THEN `_chainReset()` runs but leaves `_brakedBlindCarry*` intact, so the post-chain carry survives the retire.

### Stub-guard (owner: **T4**) — dispatch-only

At `PathTracker.cpp:3386`, change the final-hop stub PWM from the plan start speed to the floor:

```cpp
// was: const int16_t stubPwm = chainMid ? chainEndPwm : signedStartSpeed;
const int16_t floorPwm = decision.reverse ? (int16_t)-Config::PWM_MIN_MAGNITUDE
                                           : (int16_t) Config::PWM_MIN_MAGNITUDE;
const int16_t stubPwm  = chainMid ? chainEndPwm : floorPwm;   // final hop crawls at floor, not plan PWM
```

A mid-chain stub keeps flowing at its planned exit (`chainEndPwm`); only the **final** (braked) stub changes. Dispatch detail only — the `#ifdef PT_PARITY_TRACE` emit (`3548-3556`) still records `signedStartSpeed`/`0`, so the trace is unchanged (mirrors the existing `chainFinalEndPwm` floor pin). Applies to both the GYRO (`3390-3398`) and SYNC (`3402-3407`) stub arms — verify both use the guarded value.

### Shared carry helper (owner: **T5**) — mandatory

Extract the `traveledCm`/`carryCm` computation currently inline in the AC9 block (`3263-3270`) into ONE private helper that both the in-chain path and the generalized path call. A copy is NOT acceptable — a single artifact is required so the two paths cannot drift (`gotcha-drive-constant-retune-fanout` family).

### Unchanged contracts (must NOT drift)

- Parity schema (`sim/parity.py` `blind_dead_reckon` = `{node, mode, dist_cm, start_pwm, end_pwm, chain, reanchor}`, `blind_pass = {node}`) and `PathTrackerParityTrace.h` — **no field add/change**.
- `ChainSpeedPolicy.h` / `chain_speed.py` / `chain_speed_goldens.json`, CTPS `track_builder.py`, `routes.h/.cpp`, `field_data.*` — **untouched**.

## Testing Strategy

**Levels:** Host C++ ctest (dispatch-setpoint truth), Python parity/pytest (unchanged-trace proof), Bench (the real acceptance).

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | Chained final hop forced below `TSHAPE_MIN_MOVE_CM` via seeded carry | Host | `lastMoveSyncL/R == ±PWM_MIN_MAGNITUDE` (floor), NOT plan start (~67); `lastBrake == true`. Also assert a **mid**-hop stub still dispatches `chainEndPwm` (AC3) |
| TC2 | Chaining OFF, single goTo, ≥3 consecutive braked blind hops each with a seeded carry (`driveChainHopExact`-style) | Host | Each next command reduced by the measured carry; final node on-node; arrival step-count bounded (AC4) |
| TC3 | Carry reset across a **turn** between two braked blind hops | Host | Second hop commands FULL distance (AC5) |
| TC4 | Carry reset across a **non-blind (LINE/seek)** hop, and across a **fault→clear** | Host | Following braked blind hop commands full distance (AC5) |
| TC5 | Carry reset across **IDLE / fresh-goTo origin bind**, and across a **wall/line re-anchor** | Host | Full distance on the next braked blind hop; and (origin-bind) the within-goTo continuation does NOT reset (AC5) |
| TC6 | **Direction flip** — a REAR braked blind hop after a FRONT one with no turn | Host | Carry NOT subtracted (carry=0), re-armed for REAR; landing correct, not ~2×carry short (AC5) |
| TC7 | **Chain-divergence fallback** dispatch (`3288-3305`) | Host | Subtracts ZERO stale carry; re-snapshots a fresh baseline (AC5) |
| TC8 | **Chain-final → post-chain handoff** — chain retires, a following non-chain braked blind hop telescopes | Host | Post-chain hop reads the chain-final carry; no double-subtract; on-node (AC5) |
| TC9 | Existing in-chain carry (current TC8 in `test_path_tracker_blind_chain.cpp`) | Host | No regression |
| TC10 | Full parity trace diff (firmware host vs sim), goldens, routes-freshness | pytest/ctest | Green with zero schema/golden/routes edits (AC7, AC8); parity-enabled pytest green after T1 |
| TC11 | Bench: `goTo("MWU")` chaining ON, and the same single goTo chaining OFF | Bench | Land <1 cm from MWU, clear of wall (AC1, AC2) — the fallback-decision gate |

**Test data:** reuse the `MiniR4.DriveDC` mock (`lastMoveSyncL/R`, `lastBrake`, `brakeCallCount`) and the `driveChainHopExact(commandedCm, carryCm)` carry-seeding helper in `test_path_tracker_blind_chain.cpp:243`. New cases go in that file (or a sibling `test_path_tracker_blind_carry.cpp` with its own CMake target + `add_fixture_routes_force_include` + `add_path_force_includes`).
**Run commands:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`; `uv run --extra dev pytest tools/path_planner/tests/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Re-enable blind chaining | — | low | `WRO2026/config/ConfigNav.h` | Flip `PATH_BLIND_CHAIN_ENABLED_DEFAULT` `false`→`true` (line 709); drop any temp comment. Heals the currently-red parity pytest (Decisions). Satisfies AC6, part of AC7. |
| T2 | Restore `testOne` | — | low | `WRO2026/Tasks.cpp` | Replace the diagnostic body (338-339) and the commented instrumented climb (320-337; keep banner intent out) with the original 8-node climb preserved at 341-362; remove consumed comment scaffolding. Satisfies AC6. |
| T3 | Remove `[HOP]` instrumentation | — | low | `WRO2026/src/mission/PathTracker.cpp` | Delete the `// TEMP BENCH INSTRUMENTATION` `Serial` block (3355-3373). Same-file with T4/T5 → runs first in the PathTracker chain. Satisfies AC6. |
| T4 | Stub-guard: final degraded hop brakes from floor | T3 | med | `WRO2026/src/mission/PathTracker.cpp` | Per Contracts §Stub-guard: final-hop `stubPwm` → reverse-signed `±PWM_MIN_MAGNITUDE`, mid-hop unchanged, both GYRO+SYNC arms. Parity-invisible → correctness only via TC1. Satisfies AC3. |
| T5 | Generalize measured carry to braked blind sequences (companion D) | T4 | high | `WRO2026/src/mission/PathTracker.cpp`, `WRO2026/src/mission/PathTracker.h` | Per Contracts §New state + §Shared helper: add the 4 members; extract the shared carry helper (mandatory); implement the SNAPSHOT / READ / DIRECTION-FLIP / INVALIDATE / origin-bind / chain-retire-ordering state machine exactly as specified; `_chainReset()` must not touch the members. Satisfies AC4, AC5. |
| T6 | Host ctests | T4, T5 | med | `WRO2026/test/test_path_tracker_blind_chain.cpp` (or new sibling), `WRO2026/test/CMakeLists.txt` | Implement TC1–TC8 asserting the **real** `MiniR4.DriveDC` setpoint (not the plan). If a new file, add the CMake target + force-includes. Satisfies AC3-AC5. |
| T7 | Regression + trace-invariance verification | T1, T2, T3, T5, T6 | low | (run-only) | Full `ctest` + full `uv run --extra dev pytest`; confirm the parity pytest is green (needs T1), parity job green with zero schema/golden/routes edits, `git status` shows no `routes.*`/goldens/parity changes. Satisfies AC7, AC8. |
| T8 | ADR-079 amending ADR-059 | — | low | `docs/decisions/079-braked-blind-carry-and-stub-guard.md`, `DECISIONS.md`, `docs/decisions/059-collinear-run-continuous-motion.md` | New ADR: carry generalized beyond chains; final-hop degraded-stub floor pin; fallback ordering (terminal reserve, then coalescing) recorded against ADR-059's revisit note + "amended by ADR-079"; bump DECISIONS.md index + status count. May draft in parallel once the T4/T5 design here is final (this spec). Satisfies AC9. |

Parallelism: T1, T2, T8 run parallel to the PathTracker chain. T3→T4→T5 are serialized (same file). T6 waits for T4+T5; T7 is the final gate (T1, T2, T3, T5, T6).

## Notes for Implementer

- **Bench gate = the real acceptance.** AC1/AC2/AC7 (bench + full CI) decide success. Host tests prove the dispatch is *what we intend*; only the ruler at MWU proves it *fixes the overshoot*. If AC1 fails while AC2 passes, the failure is stub-side → escalate to the **terminal reserve** (first fallback), NOT Option A. If both fail or the residual looks like a per-hop *bias* (constant offset independent of hop count, not accumulating), STOP and revisit the mechanism (Decisions) before either fallback.
- **Reuse, don't re-derive, the carry arithmetic** (mandatory shared helper — Contracts §Shared helper).
- **Do not double-subtract** — the `!match` guard is the sole interlock; while the in-chain AC9 block subtracts, the generalized READ is skipped (but the SNAPSHOT still runs). Verify with TC8.
- **Parity-invisible ⇒ test the setpoint.** Per `gotcha-parity-ctps-record-plan-not-dispatch`: the stub-guard and the carry both vanish from the parity/sim/CTPS trace. TC1-TC8 MUST assert `lastMoveSyncL/R` (and `lastBrake`), not the recorded plan — a green parity job is NOT evidence either works.
- **RAII builder rule** — any Drive builder edit stays one discarded-temporary `.async()` statement (`gotcha_drive_builder_raii`); don't bind a builder to a local.
- **`SYNC` is broken on this chassis** (crossed encoder cables); the column runs `GYRO`. The stub-guard and carry state are mode-agnostic; verify both SYNC and GYRO branches.
- **Terminal-reserve fallback detail** (if invoked): plan the floor to arrive `R ≈ 4 cm` (> worst measured carry ~3 cm) before the run end — `terminal entry = floor + accel·max(0, d_final − R)` in `ChainSpeedPolicy.h`'s backward pass — so the post-carry residual stays ≥ `TSHAPE_MIN_MOVE_CM` and takes the profiled path. Re-pin `chain_speed_goldens.json` + the `chain_speed.py` mirror; parity record shape unchanged. This is the variant that leverages Diego's "short *profiled* hops brake accurately" fact directly.
- **Approach provenance.** Fable (approach consult) vetoed the original Option 1 (coalescing), citing ADR-059's explicit prior rejection + the verified carry-blind/stub root cause; the carry-generalization + stub fix that became this plan is Fable's recommendation minus the terminal reserve (dropped on Diego's short-hop-accuracy bench fact, retained as first fallback). Fable's spec critic then raised the 3 criticals (carry-state lifecycle: reset-vs-AC2, direction-flip, double-subtract ordering) folded into Contracts above via the Opus judge (C1/C2 CRITIC_WINS, C3 SYNTHESIS). Codex consult/critic unavailable (forwarder returned an unretrievable background task handle both times).
