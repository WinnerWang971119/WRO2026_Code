# Collinear-Run Continuous Motion (whole-run trapezoid + CTPS truth)

**Goal:** Make a `goTo()` over a collinear chain of nodes drive as ONE continuous motion — a single ramp-up at the start, cruise through the intermediate nodes, and a single ramp-down/stop at the end — instead of stopping and re-accelerating at every node; and make the CTPS player render that truthfully.

**Approach:** Reframed after a cross-vendor architecture consult + a dual adversarial critic pass (both read the code). Findings that drive the shape: (1) **LINE-junction collinear chains already produce the whole-run trapezoid in firmware** when pass-through fires — the stop is `cruise→floor(50)→brake` (ADR-058), so shed-to-floor is `(85−50)/5 = 7 cm`, which fits the ~12 cm final overshoot window; no cross-hop ramp arming is needed. (2) **PROJECTION collinear chains do NOT hold cruise today** — `PROJECTION_FOLLOW` arms `setBrakeOnGate(false)` but never `setTShapeRampDown(false)`, so each projection hop sheds 85→50 then snaps back; this needs a one-line firmware fix or CTPS would ship a *new* lie. (3) **CTPS is already lying** for line/projection pass tags (`end=0` in `track_builder`). (4) **BLIND chains need the real new firmware code** — standard **forward+backward-pass velocity chaining** (the GRBL/Marlin scheme): keep one command per hop, chain `setStartSpeed/setEndSpeed/setBrake(false)` so the whole-run trapezoid falls out with **no new cross-hop controller state**; the final hop stays an ordinary `cruise→floor→brake` braked stop. The chain-speed math is a header-only `ChainSpeedPolicy.h` (the ADR-058 policy pattern), mirrored once in Python, golden-pinned; the sim *emits* the resolved per-hop speeds and CTPS *consumes* them — CTPS never re-derives a chain decision.

## Scope

- **In scope:**
  - **Firmware projection ramp-down fix:** chain `setTShapeRampDown(!projPassThrough)` in the `PROJECTION_FOLLOW` arm so a `line–proj–line` pass-through actually holds cruise (prereq for a truthful CTPS render).
  - **CTPS truth-fix (run-grouper):** merge a whole collinear run into ONE velocity profile so a pass-through run renders as one trapezoid (line + projection; blind added after the firmware lands).
  - **`ChainSpeedPolicy.h`** (firmware, header-only, pure) + Python mirror + cross-language golden vectors: the forward+backward per-hop `(startPwm, endPwm)` math, with an `entryPwm` parameter.
  - **Firmware `PathTracker` blind-chain chaining:** walk-ahead, resolve chain speeds, dispatch mid-hops with chained start/end + `setBrake(false)` (final hop = ordinary braked stop), subtracting the measured inter-hop boundary carry from each next hop's commanded distance.
  - **Guards:** `PATH_BLIND_CHAIN_ENABLED` kill-switch (default ON), `PATH_BLIND_CHAIN_MAX_CM` run cap (sized ≤ the `STOP_AT_LINE_ARM_WINDOW_CM` error budget), `MAX_CHAIN_HOPS` compile-time cap; chain-ineligible at pivot / seek / reset / goal nodes, direction flips, non-collinear bends, and hops `< TSHAPE_MIN_MOVE_CM`.
  - **Sim + parity mirror:** the `ACT_BLIND_PASS` primitive + chain-eligibility + the resolved speeds into the ADR-050 parity record (strict-schema field-set bump) and the CTPS command-stream (existing `start_speed`/`end_speed` fields).
  - New ADR (amends ADR-023/046/052/058) + tests (host ctest, CTPS/sim pytest, parity sweep, golden vectors, a new blind-chain fixture).

- **Out of scope:**
  - **`NAV_TURN_TOLERANCE_DEG` stays 0.5°** — no tolerance change, no new pass-through tolerance constant. Diego authors the field nodes truly collinear (within 0.5°) in parallel; we do **not** edit `field_data.json`. The feature is validated by the `collinear_4node` fixture + a new blind-chain fixture, NOT by the CY1→CY3 path.
  - Run-*coalescing* into a single multi-hop Drive command (rejected — see Decisions for the recorded counterweights).
  - Any new mid-run positional re-anchor for blind chains (blind nodes have none today; we do not invent one).
  - Mission/task authoring, manipulator, real competition mission content.

## Decisions

- **Reframed mechanism (both consultants + both critics):** no run-scoped ramp-down distance accounting inside `LineFollower`/`Drive`; run-scoping lives in `PathTracker`'s walk-ahead + the pure `ChainSpeedPolicy`. Cross-hop controller state is what ADR-050 warns against widening.
- **CTPS-truth-first:** the run-grouper (line + projection) is low-risk and repairs an already-shipped falsehood; it lands with only the small projection firmware fix as a prerequisite, before the blind firmware work.
- **Blind chains: IN, high priority.** Diego's bench experience: every stop→launch is a static-friction break-away that spikes heat + slips the wheels, corrupting the next move's heading; blind hops have no line to wash it out, so a continuous run dead-reckons *better* than N stop-starts. Blind nodes have NO positional re-anchor today — stopping there corrects nothing (memory: `project-fewer-stops-better-friction-drift`).
- **Guarded, not gated-off:** default ON behind `PATH_BLIND_CHAIN_ENABLED` + `PATH_BLIND_CHAIN_MAX_CM` + `MAX_CHAIN_HOPS`; bench verification is Diego's.
- **`ChainSpeedPolicy` is a two-pass junction planner (the GRBL/Marlin precedent the spec cites):** a forward (accel-limit) pass AND a backward (decel-to-floor) pass, clamped to `[pwmMin, |cruise|]`, with an `entryPwm` seed. The final hop is the ORDINARY `cruise→floor→brake` stop (`end=0`, brake) — the backward pass guarantees the entry to the final hop is low enough that the 7 cm shed-to-floor fits, pulling the shed into the prior hop when the final hop is short. This avoids the half-clamp "slam" (the naive `min(cruise, pwmMin+accel·distToEnd)` decel-only formula produced a 17 cm ramp clamped to 5 cm → a 3.4× over-accel slam that goldens would not catch).
- **`entryPwm` preserves the ADR-023 cruise carry:** a blind chain that begins right after a LINE/projection pass-through seeds `entryPwm = captured cruise`, not `TSHAPE_START_SPEED` — otherwise the chassis (physically at 85) would be commanded start=50 and re-dip to the floor, the exact lurch this plan removes. A cold-start chain seeds `entryPwm = ±TSHAPE_START_SPEED`.
- **Ramp-down is `(cruise − PWM_MIN)/accel` (shed-to-floor), not `cruise/accel`.** The stop is `cruise→floor→brake` (ADR-058). This is why LINE chains already fit and why the final-hop end target is the floor, not 0.
- **Inter-hop boundary carry is measured and subtracted, not assumed zero.** The chassis holds the last PWM for ~2–3 ticks (~35 Hz loop) ≈ 1.5–3 cm per node boundary; that travel lands before the next hop's encoder baseline. The next hop's commanded distance is reduced by the measured encoder delta so error does not accumulate over an N-hop chain.
- **A collinear run MAY span LINE/projection → BLIND** (the ADR-023 LINE→BLIND handoff already carries cruise). The run continues across a `line→blind` transition when collinear + eligible; it ends at the first braked stop or ineligible node.
- **A chain preceding a `stopAtLine` seek** ends before the seek (seek nodes are chain-ineligible); the seek is a fresh braked hop. `PATH_BLIND_CHAIN_MAX_CM ≤ STOP_AT_LINE_ARM_WINDOW_CM` error budget so the chain's residual dead-reckon still lands within the seek's arm window.
- **Wire surfaces are two, kept distinct:** (a) CTPS command-stream — reuse the EXISTING optional `start_speed`/`end_speed` fields (already consumed by `resolve_effective_speed`), no new field; (b) ADR-050 parity record (`parity.py`, STRICT — raises `ParitySchemaError` on unknown *or missing* field of a known kind) — bump the drive/blind record kind's field set to carry resolved speeds + a `chain` flag, emitted on BOTH firmware and sim, accepting the one-time trace change. Do not conflate the two.
- **AC8 is NOT a build-failing static_assert.** The FAST + REAR-led final-hop shed (8 cm) already exceeds the 7.68 cm rear overshoot window today, and the firmware degrades gracefully (`LineFollower.cpp:287-301` fits `rampDownCm` to the available window). So the requirement is that `profile.py`'s stop kernel MODELS that window-fit (renders the steeper-but-reaches-floor shed), not that the build fails. A compile-time guard, if any, is a `@warning`, not an error.
- **Keep the single 0.5° tolerance knob** — split rejected: pass-through ⟺ skipped-turn is definitional, so a split where pass-through fires while the pre-edge turn still dispatches is incoherent. Diego keeps it tight and straightens the field.
- **Run-coalescing rejected, counterweights recorded:** one Drive command per run would remove the boundary-carry, `TSHAPE_MIN_MOVE_CM`-no-op, and forced-steep-final-decel defects that per-hop chaining must handle — but it breaks the hop-granular ADR-050 parity trace and needs a mid-move distance observer. The decision stands on parity granularity, not on those being non-issues.
- **New ADR-059** amends ADR-023/046/052/058. Verify no ADR-number collision at execute time (memory: `gotcha-adr-number-collision`).
- **Diagnostic (symptom tie-back):** CY1→CY3 expands (verified) to a LINE column CY2→CG2→CB2→C then a BLIND run C→p1→i6→i3; on the current field those bends are 1.1–2.7° so pass-through never fires (Diego straightens them). After this change: the LINE column is answered by the projection/line grouper + firmware (once straight), the blind run by blind chaining.

## Acceptance Criteria

- [ ] AC1 — On a collinear LINE-junction chain (`collinear_4node` fixture), CTPS renders ONE trapezoid: single ramp-up, constant cruise through the intermediate nodes (no dip to floor, no stop keyframes), single ramp-down/stop.
- [ ] AC2 — On a collinear PROJECTION chain, the firmware holds cruise across each projection node (after the `setTShapeRampDown(!projPassThrough)` fix) AND CTPS renders it as one trapezoid — no per-node dip (neither a stop lie nor a no-dip lie).
- [ ] AC3 — On a collinear BLIND chain, the firmware dispatches mid-hops with `setStartSpeed`/`setEndSpeed`/`setBrake(false)`; the chassis never brakes to rest at an intermediate node; the final hop is an ordinary braked `cruise→floor→brake` stop; each intermediate node is stamped once (`onDeadReckonedAdvance`).
- [ ] AC4 — `ChainSpeedPolicy` runs BOTH passes: no commanded end speed exceeds `entry + accel·hopDist` (forward) and every node speed sheds to reach the floor by the run end (backward); a short final hop pulls the shed into the prior hop; no hop is commanded a decel steeper than `accel` (no half-clamp slam).
- [ ] AC5 — `entryPwm`: a blind chain following a pass-through seeds the captured cruise (no re-dip to floor); a cold-start chain seeds `±TSHAPE_START_SPEED`.
- [ ] AC6 — `ChainSpeedPolicy.h` (C++) and `chain_speed.py` produce byte-identical `(startPwm,endPwm)` on the golden vectors (float32/int16 discipline).
- [ ] AC7 — Guards: `PATH_BLIND_CHAIN_ENABLED=false` → stops at every node (today's behavior); a run exceeding `PATH_BLIND_CHAIN_MAX_CM` or `MAX_CHAIN_HOPS` ends at the cap (that hop brakes) + logs the truncation; a pivot/seek/reset/goal node, a direction flip, a non-collinear bend, or a `< TSHAPE_MIN_MOVE_CM` hop each end the chain before the unsafe hop.
- [ ] AC8 — `profile.py`'s stop kernel models the LineFollower window-fit, so a FAST + REAR-led final hop renders the window-fitted shed (reaches the floor before the brake), NOT an unfitted 8 cm shed. No build-failing assert.
- [ ] AC9 — Inter-hop boundary carry is subtracted: the final blind node of an N-hop chain lands on-node within tolerance (no linear drift with N).
- [ ] AC10 — The sim↔firmware FSM parity job (ADR-050) is GREEN over an all-pairs `goTo` sweep including collinear line, projection, and blind chains — identical primitive stream + resolved speeds + `ACT_BLIND_PASS`.
- [ ] AC11 — CTPS consumes the recorded `start_speed`/`end_speed` for blind runs and renders their whole-run trapezoid; it never re-derives a chain decision. The CTPS run-grouper treats zero-motion records (`ACT_HEADING_RESET`, declare_position, manip/wait) as TRANSPARENT to a run, and falls back to per-primitive rendering (with a `UserWarning`) only on a genuinely malformed run.
- [ ] AC12 — Non-chain single hops (and every existing trace) render byte-identical to pre-change (`resolveChainSpeeds` applies only to `hopCount ≥ 2` eligible chains).
- [ ] AC13 — Full `pytest` + full `ctest` pass; `arduino-cli compile` passes; regenerated `routes.h/.cpp` unchanged (no field edits).

## Contracts & Interfaces

### Shared types / signatures

- **`ChainSpeedPolicy.h`** (firmware, header-only, pure — the ADR-058 `TShapeProfilePolicy.h` pattern, host-testable, no Arduino surface). Owner: **T4**. Consumers: T7 (firmware dispatch), T5 (Python mirror), T4 (host golden TU).
  - `struct ChainSpeedConfig { int16_t pwmMin; int16_t pwmMax; float accelPwmPerCm; float minMoveCm; };`
  - `struct ChainHopSpeed { int16_t startPwm; int16_t endPwm; bool brake; };  // brake true only on out[hopCount-1]`
  - `// Two-pass junction planner over a collinear run of hopCount hops (>=2; hopCount+1 nodes).`
  - `// entryPwm: signed PWM the chassis is physically at when the run begins (captured cruise after a`
  - `//   pass-through, else +/-TSHAPE_START_SPEED). cruisePwm: signed run cruise. Fills out[0..hopCount-1].`
  - `// forward:  vExit[k] <= vEntry[k] + accel*hopDist[k];  backward: vEntry[k] <= pwmMin + accel*distToEnd[k];`
  - `// clamp each node speed to [pwmMin,|cruise|]; node0=|entryPwm| clamped; final hop is the ordinary stop`
  - `// (out[last] = { startFromChain, 0, /*brake=*/true }). float32/int16 discipline (mirror profile.py _f32/_trunc_i16).`
  - `void resolveChainSpeeds(const float* hopDistCm, uint8_t hopCount, int16_t cruisePwm, int16_t entryPwm, const ChainSpeedConfig& cfg, ChainHopSpeed* out);`
- **`chain_speed.py`** (`tools/path_planner/ctps/chain_speed.py`, NEW). Owner: **T5**. Pure float32 mirror (numpy float32 + int16 truncation, exactly like `ctps/profile.py`'s kernel). Consumers: T8 (sim emit), T9 (CTPS), T11 (golden test).
- **Pure lookahead predicate** — extract the collinearity/direction test into a pure function taking EXPLICIT `(inboundBodyHeading, inboundDirection, outboundEdge)` inputs (today `_passThroughHeadingOk` reads live FSM state `_resolvedBodyHeading`/`_resolvedDirection`, so it cannot be called repeatedly for multi-hop lookahead). Owner: **T7** (firmware) + **T8** (sim mirror); the existing pass-through call site adopts it too.

### Primitive pass tags (3-place contract — re-derived from code; memory: `gotcha-ctps-drive-method-contract`)

- Existing pass tags: `ACT_LINE_PASS = "line_pass"`, `ACT_FOLLOW_DIST_PASS`, `ACT_PROJECTION_REVERSE_PASS`.
- NEW: `ACT_BLIND_PASS = "blind_pass"` (blind-chain mid-hop, non-braking). The three lock-step places are: (1) **sim emit** — `sim/runtime.py`; (2) **render** — `ctps/track_builder.py` (direct tag import + grouper handling); (3) **parity schema** — `sim/parity.py` `TRACE_RECORD_FIELDS`. **NOT** `trace_loader.py` — it whitelists `drive.method` strings (`_KNOWN_DRIVE_METHODS`), not `ACT_*` tags. Owner: **T8**.

### Wire fields (two distinct surfaces — do not conflate)

- **CTPS command-stream:** reuse EXISTING optional `start_speed` / `end_speed` on the drive record, populated by the recorder for a chained hop; `resolve_effective_speed` already consumes them; the CTPS grouper reads them for the blind whole-run profile. Loader (`trace_loader.py`) already guards these (strict int-or-none). No new command-stream field. Owner: **T8** (recorder emit) / **T9** (CTPS consume).
- **ADR-050 parity record:** `parity.py` is STRICT (`ParitySchemaError` on unknown *or missing* field of a known kind). Bump the drive/blind record kind's field set with the resolved `start_pwm`/`end_pwm` + `chain: bool`; emit on BOTH the firmware `PT_PARITY_TRACE` side (`PathTracker.cpp` emitters + `PathTrackerParityTrace.h`) and the sim. Accept the one-time parity-trace change. Owner: **T7** (firmware emit) + **T8** (sim emit + schema).

### CTPS run-grouper contract

Owner: **T3** (line/proj), extended by **T9** (blind). A **run** is bounded by braked stops. Walking the primitive stream: accumulate a run from the first motion primitive after a stop until (and including) the next BRAKED stop. Within a run — geometry primitives (`ACT_FOLLOW_INT`, `ACT_LINE_PASS`, `ACT_FORWARD`, `ACT_BLIND_PASS`, projection follows/passes) contribute distance; a PASS tag marks a non-braking node boundary (run continues); a non-pass distance-gated stop (`ACT_LINE_OVERSHOOT`/`ACT_STOP_AT_LINE`/… without a pass tag) ENDS the run; **zero-motion records** (`ACT_HEADING_RESET`, `declare_position`, manip/wait) are TRANSPARENT (skipped, run continues). Build ONE velocity profile over the run's summed chord distance (line/proj: `resolve_drive_profile`; blind: consume recorded per-hop `start_speed`/`end_speed`), slice `sample_drive` at each geometry primitive's cumulative-distance boundary. **Invariant (TC7):** each primitive's LAST sub-keyframe still byte-pins its sim-snapshot pose — only timing/velocity BETWEEN pose boundaries changes.

### Config knobs (mirrored firmware ↔ sim, regex-parity like `PATH_PASS_THROUGH_ENABLED`)

- `Config::PATH_BLIND_CHAIN_ENABLED` (bool, default `true`), `Config::PATH_BLIND_CHAIN_MAX_CM` (float, ≤ arm-window budget), `Config::MAX_CHAIN_HOPS` (uint, e.g. 8) — all in `ConfigNav.h`, each mirrored as a `runtime.py` module constant with a `test_pass_through`-style regex-grep parity test. Owner: **T6**. Consumers: T7 (firmware), T8 (sim).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `docs/decisions/059-*.md`, `DECISIONS.md` | T1 | — |
| `WRO2026/src/mission/PathTracker.cpp` / `.h`, `WRO2026/src/mission/PathTrackerParityTrace.h` | T2 (projection fix) then T7 (blind chaining) — **T7 blocked by T2** | — |
| `tools/path_planner/ctps/track_builder.py`, `ctps/profile.py` | T3 | T9 |
| `WRO2026/src/drive/ChainSpeedPolicy.h`, `WRO2026/test/CMakeLists.txt` (golden TU stanza) | T4 | T7, T10 |
| `tools/path_planner/ctps/chain_speed.py` (new) | T5 | T8, T9 |
| `WRO2026/config/ConfigNav.h`, `ConfigChecks.h` | T6 | T7, T8 |
| `tools/path_planner/sim/runtime.py`, `sim/parity.py` | T8 | T9 |
| `tools/path_planner/ctps/trace_loader.py` | T8 | T9 |
| `WRO2026/test/test_path_tracker_blind_chain.cpp` (new), `WRO2026/test/mocks/Drive.h` (add builder-knob recording) | T10 | — |
| `tools/path_planner/tests/**` (fixtures + unit/parity tests) | T11 | — |

## Data Model

- **Firmware run buffer (fixed-size, no heap):** a `PathTracker` member array sized `MAX_CHAIN_HOPS` holding walked-ahead `hopDistCm[]` + resolved `ChainHopSpeed[]`. Overflow (run > `MAX_CHAIN_HOPS` or `PATH_BLIND_CHAIN_MAX_CM`) ends the chain at the cap (that hop brakes) and a fresh chain re-forms after.
- **CTPS command-stream:** existing optional `start_speed:int`/`end_speed:int` populated on chained drive hops. Absent ⇒ resolver default (non-chain unchanged).
- **ADR-050 parity record (additive, field-set bump):** the drive/blind record kind gains `start_pwm:int`, `end_pwm:int`, `chain:bool`; both surfaces emit them (strict schema — not "absent → default").

## Error Handling

- **Chain over cap / `MAX_CHAIN_HOPS`:** not a fault — end at the cap (brake), re-form after; `log()`/Serial the truncation (no silent cap).
- **Ineligible node mid-walk** (pivot/seek/reset/goal, direction flip, non-collinear, `< TSHAPE_MIN_MOVE_CM` hop): end the chain before it; that hop is a normal braked hop.
- **`ChainSpeedPolicy` degenerate inputs:** `hopCount < 2` ⇒ not a chain (caller uses the ordinary per-hop path); zero/negative hop distance floors to 0; `accel <= 0` clamps to a tiny positive slope. Never divide by zero.
- **CTPS grouper meets a genuinely malformed run** (pass tag with no following braked stop, or a blind run missing recorded speeds): reject the group → fall back to per-primitive rendering + `UserWarning`. Zero-motion records are NOT malformed — they are transparent.
- **Parity: unknown OR missing field of a known kind** → `ParitySchemaError` (loud) — hence the field-set bump must land on both sides together.

## Testing Strategy

**Levels:** Unit (ChainSpeedPolicy golden, CTPS grouper), Integration (host FSM ctest, sim↔firmware parity), Golden (cross-language vectors).

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | `ChainSpeedPolicy` C++ vs Python goldens: even [10,10,10]; short final hop [10,10,3]; short MID hop [10,3,10]; single-hop (rejected, hopCount<2); reverse cruise; `entryPwm=cruise` vs `=start` | Unit/Golden | Byte-identical; forward+backward both enforced; no decel steeper than `accel`; short final hop pulls shed into prior hop; single-hop takes the ordinary path |
| TC2 | Collinear 4-node LINE fixture through CTPS grouper | Unit | ONE trapezoid; zero intra-run stop keyframes; each primitive's last sub-keyframe pins its snapshot pose (TC7) |
| TC3 | Collinear PROJECTION chain: firmware `setTShapeRampDown(false)` fix + CTPS render | Integration+Unit | Firmware holds cruise across projection nodes; CTPS shows no per-node dip |
| TC4 | Collinear BLIND chain, enabled | Integration (host ctest) | Mock records mid-hop `setBrake(false)` + chained start/end and NO `Drive.stop()` between hops; final hop braked; each node stamped once |
| TC5 | Blind chain, `PATH_BLIND_CHAIN_ENABLED=false` | Integration | Stops at every node (today's behavior) |
| TC6 | Blind run over `PATH_BLIND_CHAIN_MAX_CM` / `MAX_CHAIN_HOPS`; and each ineligible reason (pivot, seek, reset, goal, direction flip, non-collinear, `<5cm` hop) | Integration | Chain ends before the unsafe hop (it brakes); truncation logged (assert log content) |
| TC7 | LINE→BLIND spanning collinear run | Integration | One run across the transition; `entryPwm=captured cruise` on the first blind hop (no floor dip) |
| TC8 | Inter-hop boundary carry over an N-hop chain | Integration | Next hop's commanded distance reduced by measured carry; final node on-node within tolerance, no drift with N |
| TC9 | FAST + REAR-led final hop shed vs rear window in `profile.py` | Unit | Renders the window-fitted shed (reaches floor before brake); NO build-failing assert |
| TC10 | sim↔firmware parity all-pairs `goTo` incl. line/proj/blind chains | Integration (ADR-050) | Identical primitive stream + resolved speeds + `ACT_BLIND_PASS`; strict schema round-trips |
| TC11 | CTPS malformed-run fallback (pass-without-stop; blind missing speeds) AND zero-motion interleave (`ACT_HEADING_RESET` mid projection run) | Unit | Malformed ⇒ per-primitive fallback + `UserWarning`; heading-reset interleave ⇒ run stays grouped (transparent) |
| TC12 | Non-chain single hop + an existing trace (regression) | Unit/Integration | Byte-identical to pre-change |

**Test data:** `collinear_4node` fixture (line); NEW collinear-projection + collinear-blind fixtures; a `routes_fixture`/`field_fixture` update (+ regenerate + commit) so the ADR-050 all-pairs sweep actually contains line/proj/blind chains (owned by T11); golden vectors dumped from a host `ChainSpeedPolicy` TU. **Run:** `uv run --extra dev pytest tools/path_planner/tests/`; `ctest --test-dir build/test --output-on-failure`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | ADR-059 + index | — | low | `docs/decisions/059-collinear-run-continuous-motion.md`, `DECISIONS.md` | ADR amending ADR-023/046/052/058: forward+backward chain planning; projection ramp-down fix; CTPS grouper; keep 0.5°; guards; two wire surfaces. Update index + status table. Verify no ADR-number collision (memory `gotcha-adr-number-collision`). |
| T2 | Firmware projection ramp-down fix | — | med | `WRO2026/src/mission/PathTracker.cpp` | In the `PROJECTION_FOLLOW` arm (`_dispatchEdgeAfterTurn`, ~PathTracker.cpp:2281-2286) chain `.setTShapeRampDown(!_projPassThrough)` so a `line–proj–line` pass-through holds cruise instead of shedding 85→50 per hop. VERIFY the exact arm site first. Satisfies AC2 (firmware half). |
| T3 | CTPS run-grouper (line + projection) | T2 | high | `tools/path_planner/ctps/track_builder.py`, `tools/path_planner/ctps/profile.py` | Fix `_primitive_drive_speed` so pass tags (`ACT_LINE_PASS` + BOTH projection pass tags — all currently `end=0`) aren't forced to stop; add the run-grouper per the Contracts (run bounded by braked stops; follow legs included; zero-motion records transparent; ONE `DriveProfile` over summed distance; slice `sample_drive` at boundaries preserving pose pins). Standalone on existing line/proj tags. Satisfies AC1, AC2, TC2, TC11(interleave). |
| T4 | `ChainSpeedPolicy.h` + host golden TU | — | high | `WRO2026/src/drive/ChainSpeedPolicy.h`, `WRO2026/test/CMakeLists.txt` (golden stanza) | Header-only two-pass `resolveChainSpeeds` per the Contracts signature (forward accel pass + backward decel-to-floor pass, `entryPwm` seed, final hop ordinary braked stop, `hopCount>=2`, float32/int16). Host TU dumping goldens; own the golden CMake stanza. Satisfies AC4, AC5, AC6, TC1. |
| T5 | Python mirror `chain_speed.py` + golden parity | T4 | high | `tools/path_planner/ctps/chain_speed.py` (new), `tools/path_planner/tests/test_chain_speed.py` (new) | Pure float32 mirror of `resolveChainSpeeds`; golden-vector parity vs T4's dump. Satisfies AC6, TC1. |
| T6 | Config knobs + parity + AC8 model | — | med | `WRO2026/config/ConfigNav.h`, `WRO2026/config/ConfigChecks.h` | Add `PATH_BLIND_CHAIN_ENABLED` (true), `PATH_BLIND_CHAIN_MAX_CM` (≤ `STOP_AT_LINE_ARM_WINDOW_CM` budget), `MAX_CHAIN_HOPS` with per-constant `@warning` + tripwire (memory `project-config-constant-tripwire`); NO build-failing FAST/REAR assert (AC8 is a `profile.py` render requirement — see T3/T9). Satisfies AC7(config), AC8(no-assert). |
| T7 | Firmware PathTracker blind-chain chaining | T2, T4, T6 | xHigh | `WRO2026/src/mission/PathTracker.cpp`/`.h`, `WRO2026/src/mission/PathTrackerParityTrace.h` | Extract the pure lookahead predicate (explicit inbound heading/direction) + adopt it at the existing pass-through site; walk-ahead building the fixed `MAX_CHAIN_HOPS` run buffer with guards (Error Handling); call `ChainSpeedPolicy` with `entryPwm` (captured cruise after a pass-through else start seed); dispatch mid-hops `driveDist(...).tShape().setStartSpeed().setEndSpeed().setBrake(false)`, final hop ordinary braked stop; subtract measured boundary carry from each next hop's commanded distance; stamp `onDeadReckonedAdvance` per node; emit `ACT_BLIND_PASS` + resolved speeds + `chain` into `PT_PARITY_TRACE`. Satisfies AC3, AC5, AC7, AC9, TC4–TC8. |
| T8 | Sim + parity mirror | T5, T6, T7 | high | `tools/path_planner/sim/runtime.py`, `tools/path_planner/sim/parity.py`, `tools/path_planner/ctps/trace_loader.py` | Mirror the pure lookahead predicate + blind-chain eligibility; emit `ACT_BLIND_PASS` (3 places: runtime emit, track_builder import handled in T9, parity schema here) + populate command-stream `start_speed`/`end_speed` (via `chain_speed.py`) + bump the strict parity record field set (`start_pwm`/`end_pwm`/`chain`); regex-parity tests for the 3 new Config knobs. Satisfies AC10, TC10. |
| T9 | CTPS consume chain speeds + blind-pass | T3, T8 | high | `tools/path_planner/ctps/track_builder.py`, `tools/path_planner/ctps/trace_loader.py`, `tools/path_planner/ctps/profile.py` | Extend the T3 grouper to include `ACT_BLIND_PASS` and to build the blind whole-run profile from the recorded `start_speed`/`end_speed` (never re-derive); `profile.py` stop kernel models the LineFollower window-fit (AC8). Satisfies AC8, AC11, TC9, TC11(fallback). |
| T10 | Host C++ tests + mock recording | T4, T7 | med | `WRO2026/test/test_path_tracker_blind_chain.cpp` (new), `WRO2026/test/mocks/Drive.h`, `WRO2026/test/CMakeLists.txt` (blind-test stanza) | Add builder-knob recording (`setStartSpeed`/`setEndSpeed`/`setBrake`) to mock `Drive.h` (record + test-set, no FSM modeling — ADR-049; memory `gotcha-ctps-mock-noop-setter-drops-builder-knob`); ctest for the blind-chain FSM (TC4–TC8) asserting recorded knobs + no inter-hop `Drive.stop()`; wire the golden TU (from T4). Link real `PathTracker.cpp` + a toy chain routes fixture if the field lacks one (memory `project-host-test-link-toy-routes`, `gotcha-host-tests-link-production-routes`). Satisfies AC3/AC7/AC9 verification. |
| T11 | Python/CTPS/parity tests + fixtures | T5, T8, T9 | med | `tools/path_planner/tests/test_pass_through.py`, `tools/path_planner/tests/test_ctps_*.py`, `tools/path_planner/tests/fixtures/**`, `field_fixture`/`routes_fixture` | New collinear-projection + collinear-blind fixtures; ensure the ADR-050 `field_fixture` contains line/proj/blind chains (regenerate + commit `routes_fixture`); grouper unit tests (line/proj/blind, zero-motion interleave, malformed fallback — TC2/TC11); guard tests (TC6); parity sweep coverage (TC10); regression (TC12). Satisfies AC1, AC10, AC11, AC12, AC13. |

## Notes for Implementer

- **Do NOT touch `field_data.json` or `NAV_TURN_TOLERANCE_DEG`.** Diego owns field geometry and is straightening the CY columns in parallel. Validate with fixtures.
- **Ramp-down is `(cruise − PWM_MIN)/accel` (shed-to-floor), NOT `cruise/accel`.** The final hop is the ordinary `cruise→floor→brake` stop; the backward pass makes its entry low enough that the 7 cm shed fits (pulling into the prior hop for a short final hop). Sizing anything against `cruise/accel` (17 cm) is the mistake that produced the half-clamp slam.
- **`ChainSpeedPolicy` MUST run both passes** (accel-limit forward + decel-to-floor backward), like GRBL/Marlin. A decel-only formula passes the goldens but ships a steeper-than-`accel` slam — the exact "green tests, wrong motion" trap. Goldens must assert the forward constraint too.
- **`entryPwm` is load-bearing:** after a LINE/projection pass-through the chassis is at cruise, not the floor — seed the captured cruise or you re-introduce the floor-dip lurch (regressing ADR-023).
- **The projection dip is a real firmware gap** — `PROJECTION_FOLLOW` chains `setBrakeOnGate` but not `setTShapeRampDown` (verify at `_dispatchEdgeAfterTurn` ~2281). Without T2, T3's projection render is a NEW lie.
- **No cross-hop state in `LineFollower`/`Drive`.** All run-scoping lives in `PathTracker` + the pure `ChainSpeedPolicy`. The inter-hop PWM-hold handoff has direct precedent (the ADR-023 LINE→BLIND cruise carry, `PathTracker.cpp:2357-2371`) — but the boundary-carry travel MUST be measured and subtracted (it lands before the next encoder baseline; it is NOT self-cancelling).
- **Two wire surfaces, kept apart:** CTPS command-stream reuses existing `start_speed`/`end_speed` (loader already guards them); the ADR-050 parity record is the STRICT one (`parity.py` raises on unknown *or missing*) — bump the record kind's field set on BOTH sides in one change. The `ACT_BLIND_PASS` 3-place contract is runtime-emit / track_builder-render / parity-schema — NOT `trace_loader` (which whitelists `drive.method`, not `ACT_*`).
- **Grouper: a run is bounded by braked stops**, not "all non-final primitives are pass tags" (that misses the `ACT_FOLLOW_INT` follow legs and dies on `ACT_HEADING_RESET` interleaves). Zero-motion records are transparent.
- **Host mock discipline:** the blind-chain assertions reduce to "mock recorded `setBrake(false)`/chained speeds, no inter-hop `Drive.stop()`" — the mock `Drive.h` builder must RECORD those knobs (a no-op setter is a documented false-green; memory `gotcha-ctps-mock-noop-setter-drops-builder-knob`, ADR-049).
- **Golden/float discipline:** numpy float32 + int16 truncation exactly like `ctps/profile.py` (`_f32`, `_trunc_i16`); no `//`-floor of a signed divide.
- **Run the FULL ctest + FULL pytest** (contract changes ripple through fixtures — memory `gotcha-field-contract-change-breaks-cpp-and-py-fixtures`, `gotcha-strict-emitter-field-fixtures`); host + arduino-cli toolchains are local (`host_cpp_toolchain_available`, `arduino_cli_headless_available`).
- **Rollback:** `PATH_BLIND_CHAIN_ENABLED=false` restores stop-at-every-blind-node; the CTPS grouper falls back to per-primitive rendering on a malformed run.
- **Sequencing:** T2+T3 (projection fix + CTPS grouper) can land + be eyeballed first to confirm line/proj chains cruise before the blind firmware (T7) lands.
