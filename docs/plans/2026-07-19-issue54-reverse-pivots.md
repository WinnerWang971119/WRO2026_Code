# Reverse (REAR-traversal) single-wheel pivots (#54) Plan

**Goal:** Let a single-wheel pivot run when its edges are traversed in **reverse** (REAR), not just forward — closing issue #54 and lifting the FRONT-only restriction baked into ADR-036 / ADR-044 / ADR-054.

**Approach:** *Runtime flip via a shared header-only policy, with a force-SAME direction latch.* The generated `PivotTurn[]` table is **unchanged** (it still stores the forward-sense wheel + direction-invariant `δ`), so `routes.h`, its golden, and the pivot-free real field are byte-identical. A new header-only policy (`ReversePivotPolicy.h`, twinned in `sim/runtime.py`) maps `(stored forward wheel, pivot direction) → (reverse-corrected wheel, direction-resolved overshoot budget)`. Firmware, the Python sim, and CTPS all consume that one policy so the flip cannot drift across surfaces. Force-FRONT is **replaced** (not merely deleted) by **force-SAME**: a pivot commits to the direction it *approached* in, and its departure edge is forced to that same direction — the direct generalization of today's force-FRONT (which forced both legs FRONT). A binding higher-precedence override on the departure that truly contradicts the approach direction hard-faults (`PIVOT_DIRECTION_MISMATCH`); the validator rejects any statically-visible contradiction. The overshoot budget branches on whether the pivot's sensor-terminated approach is reverse-reachable. Ships as **CI-tested logic**; the physical B3.8-reverse bench + `PIVOT_SCRUB_FACTOR` are deferred until the rear board is wired (mirroring how reverse stop-at-line, B3.7-reverse, shipped).

## Scope

- **In scope:**
  - Firmware: replace force-FRONT with **force-SAME** (departure direction forced to the pivot's approach direction); flip the pivot wheel for a REAR pivot; select the direction-resolved overshoot budget; hard-fault (`PIVOT_DIRECTION_MISMATCH`) when a binding departure override contradicts the approach direction; close the 1-board REAR-*seek*-into-a-pivot serviceability gap (pivot-scoped, without regressing non-pivot forced-REAR seek behavior).
  - Offline validator: accept authored `direction:"rear"` on pivot legs; reject a **statically-visible contradictory** pivot (binding pins that force the two legs to opposite directions); budget `δ` against the REAR offset when the pivot is reverse-reachable, else the FRONT offset; **reject a REAR-pinned pivot leg on a 1-board chassis** (it would fault, not run FRONT); reject a reverse-reachable **reset-node (wall-zero)** pivot unless FRONT-pinned; treat seek-classified BLIND legs like LINE legs for reverse-reachability; fold the **pipeline direction default** into "statically FRONT-pinned".
  - Python sim: mirror the wheel-flip, the direction-resolved budget, and force-SAME; expose the taken pivot + its resolved direction for CTPS; keep firmware↔sim parity.
  - CTPS: render a REAR pivot arc (consume the sim-exposed direction through the same policy; `pivots[]` gains a `direction` field inside the non-empty-gated list — byte-stable for pivot-free traces).
  - Tests: a dedicated reverse-pivot toy fixture (authored-REAR, AUTO-REAR with a `>90°` approach delta, a `>90°` AUTO pivot that must RUN coherently, a binding-contradiction case, a reset-node case, a chained case) across validator pytest, sim↔firmware parity, host C++ FSM, and a CTPS reverse-arc golden; a world-frame geometry proof; a commanded-delta sign tripwire.
  - Docs: new ADR-071 (reverse pivots); update the FRONT-only "trigger to revisit" clauses in ADR-036/044/054; Roadmap B3.8-reverse gate + Risk #9 note; `DECISIONS.md` index; close #54.

- **Out of scope:**
  - The physical **B3.8-reverse** bench and `Config::PIVOT_SCRUB_FACTOR` — deferred to hardware bring-up (rear board not yet wired). Documented as a gate; no scored mission uses a reverse pivot until it passes.
  - Changing the pivot **authoring model** (per-turn `PIVOT_TURNS` stays) or the `PivotTurn` **struct/schema** (no routes.h fanout).
  - Retuning the `δ = (T/2)·tan(|θ|/2)` magnitude (reused as-is; direction-invariant).
  - `δ > offset` sharper-than-budget pivots reached forward — still the existing offline reject + issue #75 reverse re-seat (unchanged).

## Decisions

- **Runtime flip via shared policy** (chosen over an offline per-sense table and over inlined per-call-site flips). Keeps the `PivotTurn` schema/golden byte-identical and single-sources the flip. Endorsed by the Fable approach consult; the two rejected options add schema fanout or cross-language drift.
- **A pivot is single-direction, enforced by force-SAME.** The pivot commits to the direction it *approached* in; the departure edge is forced to that same direction. This is the generalization of today's force-FRONT (which forced both legs FRONT). δ and the flipped wheel are then always valid.
  - *Why force-SAME, not skip-on-mismatch:* under `BIDIRECTIONAL_AUTO`, every pivot with `|θ| > 90°` resolves its two legs to **opposite** directions regardless of approach (departure delta is θ after a FRONT approach, `wrap(θ−180)` after a REAR approach; the `>90°` strict threshold flips one of them). "Skip on mismatch" would therefore silently degrade **every sharp AUTO pivot** to a plain center turn — losing exactly the single-wheel clearance the pivot exists for, and landing ≈δ off because the approach already trimmed short. Force-SAME makes those pivots **run** coherently instead. There is **no** existing "FRONT-unavailable → skip" backstop to imitate (PathTracker.cpp:2085-2096: FRONT-unavailable *faults*, "no separate pivot-skip path").
- **Binding contradiction → fault + offline reject.** If a *binding* higher-precedence direction override on the departure (`direction:"…"` per-edge, or the pipeline default) truly contradicts the pivot's approach direction, the validator rejects it offline (statically visible) and firmware hard-faults `PIVOT_DIRECTION_MISMATCH` as the backstop. Ordinary AUTO never contradicts — force-SAME just pins the departure's AUTO rung.
- **Conservative per-pinning budget.** For a pivot's sensor-terminated incoming leg: if it could ever run REAR (2-board chassis, not FRONT-pinned by a per-edge pin **or** the pipeline default), budget `δ` against the REAR offset; if FRONT-pinned or reverse-impossible (sensor-in on a 1-board chassis), budget against the FRONT offset. A **REAR-pinned** sensor leg on a 1-board chassis is **rejected** (it faults at runtime, it does not run FRONT).
- **`δ > reverse-budget`: reject offline + firmware fault backstop** (`PIVOT_TRIM_UNDERFLOW`), matching ADR-044.
- **No Config flag** — force-FRONT removed unconditionally; `direction:"front"` on both pivot legs is the force-forward escape hatch (binding step-1 override in `resolveDirection`, and the FRONT budget offline). `direction:"front"` becomes newly meaningful (v1 force-FRONTs everything; post-#54 the validator branches the budget on it).
- **Pivot-scoped REAR-seek serviceability.** `reverseUnavailable` guards `EdgeType::LINE` only, but a REAR stop-at-line *seek* into a pivot on a BLIND leg also needs the rear board (ADR-027). Add a **pivot-scoped** serviceability check at the seek dispatch (fault only when `_approachingPivot.active` and the seek resolves REAR on `<2` boards) — do **not** widen `reverseUnavailable` itself, which would regress the documented non-pivot forced-REAR `STOP_AT_LINE`→dead-reckon-reverse degrade (runtime.py:483-494).
- **FRONT-pin reset-node pivots.** The wall-zero interlude drives FORWARD off the wall to `WALL_PIVOT_CLEARANCE_CM` (sized for the FRONT sweep radius, PathTracker.cpp:1409-1429); a REAR departure sweeps the tail arc near a wall — untested. The validator rejects a reverse-reachable reset-node pivot unless FRONT-pinned.
- **Symbolic budgets.** `LINE_SENSOR_AXLE_OFFSET_REAR_CM = 7.68` is an *unmeasured placeholder* (ConfigDrive.h:262-264 TODO, pending the wired rear board). All budgets in this spec are expressed via the parsed `Config` value, never a hardcoded literal; test δ values are derived from the parsed budget ± the trim floor so a future retune reddens only the four known drive-constant surfaces, not this spec's literals.
- **World-frame geometry proof, not just cross-surface parity.** Parity only proves firmware/sim/CTPS agree with each other; they could agree while all wrong. A pytest asserts the arc-end pose and the planted-wheel world-side against first-principles geometry.
- **Codex approach consult AND Codex critic both skipped** (its background runtime died mid-run each time, no output). Reconciled on the Fable consult + Fable critic alone; cross-vendor independence not obtained this round.
- **Assumptions confirmed** (Phase 3, none flagged): reverse wheel = pure L↔R flip of ADR-036's forward mapping; δ magnitude unchanged; BLIND reverse pivots run on 1 board while LINE/seek reverse pivots need 2; docs close #54 + update ADR-036/044/054 + add ADR-071; `PIVOT_SCRUB_FACTOR` stays a forward-reference.

## Acceptance Criteria

- [ ] AC1 — A pivot whose approach resolves REAR fires with the wheel **flipped** (PIVOT_LEFT↔PIVOT_RIGHT vs the stored forward wheel), the same `δ`, and its departure forced REAR; the axle lands on `w` in the sim within `_POSITION_TOL_CM`.
- [ ] AC2 — A `|θ| > 90°` pivot under `BIDIRECTIONAL_AUTO` **runs coherently** (both legs forced to the approach direction), rather than degrading to a center turn; verified in firmware (host) and sim.
- [ ] AC3 — A pivot whose departure has a **binding** override contradicting its approach direction is **rejected** offline (validator) and **faults** `PIVOT_DIRECTION_MISMATCH` at runtime (host) — never runs mis-trimmed.
- [ ] AC4 — The pivot overshoot budget uses the **REAR** offset on a reverse-reachable sensor-terminated approach and the **FRONT** offset on a FRONT approach; firmware and sim agree; the existing three direction-selects are replaced by the one policy call.
- [ ] AC5 — Validator **accepts** an authored `direction:"rear"` pivot with `δ` inside the REAR budget, and **rejects** one with `δ` inside the FRONT budget but over the REAR budget when reverse is reachable, naming the escape (pin FRONT / soften / dead-reckon leg).
- [ ] AC6 — A FRONT-pinned (`direction:"front"` on both legs) pivot with `δ` in the (REAR-budget, FRONT-budget] band **validates clean** and runs forward.
- [ ] AC7 — Validator **rejects**: (a) a REAR-pinned sensor leg on a 1-board chassis; (b) a reverse-reachable reset-node pivot that is not FRONT-pinned; (c) a binding-contradiction pivot (AC3 offline side).
- [ ] AC8 — A REAR stop-at-line **seek into a pivot** on a **1-board** chassis faults (`REVERSE_UNAVAILABLE`, pivot-scoped) and the validator treats such a leg as reverse-impossible for budgeting; a **non-pivot** forced-REAR `STOP_AT_LINE` BLIND hop still degrades to dead-reckon reverse (unchanged).
- [ ] AC9 — CTPS renders a REAR pivot as a rigid arc about the (flipped-side) planted wheel; `pivots[]` carries `direction` and the **planted (post-flip)** wheel; `Track.to_dict()` for **every pivot-free trace is byte-identical** to before this change (`schema_version` stays 1).
- [ ] AC10 — Firmware↔sim parity passes on a reverse-pivot trace; the Python policy twin matches a **frozen table of C++ `ReversePivotPolicy` outputs** (the `test_direction_parity.py` idiom — no C++ toolchain in the pytest job) for all `(wheel, direction)` inputs.
- [ ] AC11 — A world-frame geometry pytest proves the pivot **arc-end pose = `v + δ·û_out(travel)`** (~1e-6) and the trimmed departure lands on `w`, and the planted wheel is on the correct world-side, for both forward and reverse traversal; the AUTO-REAR fixture triple is asserted to actually fire the AUTO REAR rung.
- [ ] AC12 — `PathTracker.goTo` over the pivot-free real field emits an unchanged primitive/trace stream; `routes.h`/`routes.cpp` and every emitter golden are unchanged (no schema churn); full pytest + full ctest + arduino-cli compile green.

## Contracts & Interfaces

Single source of truth for every seam more than one task touches. Conform to the exact names/signatures here.

### Shared types

- `enum class Direction : uint8_t { AUTO, FRONT, REAR }` — existing, `WRO2026/src/nav/NavTypes.h`; mirrored by `sim.runtime.Direction`.
- `Routes::TurnMode { DEFAULT, …, PIVOT_LEFT, PIVOT_RIGHT }` — existing, `routes.h`; `_turnModeToByte` maps PIVOT_LEFT→1, PIVOT_RIGHT→2 (PathTracker.cpp:110-111).
- `PathTrackerClass::PivotRef` — **gains a `Direction dir` field**: `{ bool active; Routes::TurnMode wheel; float trim_cm; Direction dir; }`. `dir` = the direction resolved for the pivot's **approach** hop (`u→v`), `Direction::AUTO` by default. **Stamped after `resolveDirection` on BOTH `_approachingPivot` and `_pendingPivot`** (the carry copy is taken before the hop's direction is resolved, so the stamp must write both — else the departing copy stays `AUTO`).

### Signatures — the seam (owner: T1; consumers: T3, T4, T6, T7)

C++ header `WRO2026/src/nav/ReversePivotPolicy.h` (header-only, Doxygen'd, style-matched to `PivotTrimPolicy.h`):

- `inline Routes::TurnMode resolvePivotWheel(Routes::TurnMode storedForwardWheel, Direction dir)` — `REAR` → L↔R flip; `FRONT`/`AUTO` → unchanged; any non-pivot mode → unchanged.
- `inline float pivotOvershootBudgetCm(Direction dir)` — `REAR → Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM`; else `Config::LINE_SENSOR_AXLE_OFFSET_CM`. **This call replaces** the three existing inline direction-selects (PathTracker.cpp:1084-1086 LINE overshoot, :2693-2694 seek; runtime.py:3184-3188).

Python twin `tools/path_planner/sim/runtime.py` (its own policy-twin section, **owned by T1**): `resolve_pivot_wheel(...)`, `pivot_overshoot_budget_cm(...)`. Parity guard: `tools/path_planner/tests/test_reverse_pivot_policy_parity.py` asserts the twin matches a **frozen C++ output table** for every `(wheel ∈ {LEFT,RIGHT}, dir ∈ {FRONT,REAR})`.

### Firmware force-SAME + fault (owner: T3)

- At the departure hop, if `_departingPivot.active`: let `d = _departingPivot.dir`. If a **binding** override (per-edge `direction_override` ≠ AUTO, or a binding pipeline default) on `v→w` resolves to a direction ≠ `d` → `_enterFault(Fault::PIVOT_DIRECTION_MISMATCH, …)`. Otherwise **force** `resolved = d`. Then `wheel = resolvePivotWheel(_departingPivot.wheel, d)`.
- `Fault::PIVOT_DIRECTION_MISMATCH` — **new** fault kind + mnemonic (add to the `Fault` enum in PathTracker.h and `faultMnemonic`). `PIVOT_TRIM_UNDERFLOW` is unchanged.
- Pivot-scoped REAR-seek serviceability: at the seek dispatch, when `_approachingPivot.active` and the resolved seek direction is REAR on `<2` boards → `_enterFault(Fault::REVERSE_UNAVAILABLE, …)`. **Do not** change `reverseUnavailable`'s signature.

### Validator budget rule (owner: T7)

- `statically_front_pinned(leg) := per-edge direction pin == FORCE_FORWARD  OR  pipeline direction default == FORCE_FORWARD`.
- `board_impossible_rear(leg) := leg is sensor-terminated (LINE, or seek-classified BLIND via resolve_stop_at_line) AND LINE_BOARD_COUNT < 2`.
- `reverse_reachable(leg) := (not statically_front_pinned) and (not board_impossible_rear)`.
- overshoot budget for a sensor-terminated incoming leg `:= pivot_overshoot_budget_cm(REAR) if reverse_reachable else pivot_overshoot_budget_cm(FRONT)` (via the T1 Python twin — single source).
- **Reject** (not budget) a leg pinned `FORCE_REVERSE` when `board_impossible_rear` holds.
- Budget-vs-floor: reject when `budget − δ < PIVOT_TRIM_MIN_DRIVE_CM` (the existing floor test, validator.py:823-826 — so the boundary is `δ < budget − floor`, not `δ ≤ budget`).

### Config (owner: T0; consumers: T1, T7)

- `Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM = 7.68f` — **exists** (`ConfigDrive.h:265`); reused. Placeholder pending bench measurement (TODO at :262-264).
- `parse_line_sensor_rear_axle_offset_cm(config_path) -> float` — **new** parser in `validator.py`, mirror of `parse_line_sensor_axle_offset_cm`.

### Sim latch (owner: T4)

- The sim's approaching/pending pivot latch must hold a **by-value copy** (wrapper dataclass/tuple), never a mutable reference into `self._pivot_records` (runtime.py:2501-2518) — stamping `dir` on a shared record would corrupt the table across traversals. Mirror C++'s by-value `PivotRef`.
- Expose a **taken-pivot surface** (pivot record + resolved direction per taken pivot) for `track_builder` to consume — the direction is **derived** by the sim, not recorded in the command stream (ADR-054: recorder/`trace_loader` stay untouched).

### CTPS (owner: T6)

- `pivots[]` entries gain `"direction": "front"|"rear"` and set `wheel` to the **planted (post-flip)** wheel; both live inside the non-empty-gated `pivots` list; `schema_version` stays 1.
- The sensor-departure landing correction (`pending_pivot_land`, track_builder.py:2038-2044) derives its residual ray from a **body** heading, which points opposite travel on REAR — use the **travel-frame** departure ray for REAR (flip `depart_rad` by 180°).
- Recorder (`ctps_record.cpp`) and `trace_loader.py` are **untouched** (direction is derived from the sim, not recorded).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/src/nav/ReversePivotPolicy.h` | T1 | T3, T4, T6, T7 |
| `tools/path_planner/sim/runtime.py` — policy-twin section | T1 | T4, T7 |
| `tools/path_planner/sim/runtime.py` — FSM (force-SAME, latch, taken-pivot surface) | T4 | T5, T6, T8 |
| `WRO2026/config/ConfigDrive.h` (comment only) / `validator.py` REAR parser | T0 | T7 |
| `WRO2026/src/mission/PathTracker.h` (PivotRef.dir, `PIVOT_DIRECTION_MISMATCH`) | T2 | T3 |
| `WRO2026/src/mission/PathTracker.cpp` | T3 | — |
| `tools/path_planner/validator.py` | T7 | — |
| `tools/path_planner/ctps/track_builder.py` + `ctps/config.py` + `assets/renderer.js` + `player_html.py` | T6 | — |
| reverse-pivot fixtures (Python + host C++ routes) | T9 | T7, T8, T10, T11 |
| `WRO2026/test/fixtures/field_fixture.py` (biased parity edge) | T5 | — |
| host C++ CMakeLists + new test TUs | T10 (execute orchestrator owns CMakeLists appends) | — |
| ADR-071 / ADR-036,044,054 / Roadmap / DECISIONS.md | T12 | — |

### Naming

- New ADR = **ADR-071** (directory runs through 070; confirm no collision at authoring time).
- Authored direction tags stay `"front"`/`"rear"` → `DirectionMode::FORCE_FORWARD`/`FORCE_REVERSE`.
- New fault: `Fault::PIVOT_DIRECTION_MISMATCH` (+ mnemonic). `Fault::PIVOT_TRIM_UNDERFLOW` unchanged.

## Data / geometry model

Reverse wheel-sign, derived by mirroring ADR-036 (restated in ADR-071):

- Forward (FRONT): body rotation `θ = wrap(h_out − h_in)`; wheel `PIVOT_LEFT if θ>0 else PIVOT_RIGHT`; `δ = (T/2)·tan(|θ|/2)`.
- Reverse (REAR, force-SAME ⇒ both legs REAR): robot arrives at body heading `h_in+180`, leaves at `h_out+180`, so body rotation = `θ` (**same magnitude**); the driven wheel rolls **backward**, so the same body θ plants the **opposite** wheel → wheel **flips**; `δ` unchanged; in the world/travel frame the planted wheel is on the same side as forward.
- **Arc-end pose:** the approach lands the axle `δ` short of `v`; the rigid rotation about the planted wheel ends the axle at **`v + δ·û_out(travel)`** (δ *past* `v` along the outgoing travel ray). The departing dead-reckon trim then removes that `δ` so the chassis nets to `w`; on a sensor-terminated departure the destination line re-anchors it (ADR-054 sensor-departure re-anchor). The arc-end is **not** on `v`.

## Error handling

- **Binding contradiction** (departure binding override ≠ approach direction) → validator reject offline; firmware `Fault::PIVOT_DIRECTION_MISMATCH`. Never runs mis-trimmed.
- **`δ > reverse-budget` reached REAR** via a path the validator can't see → firmware `Fault::PIVOT_TRIM_UNDERFLOW` (existing); never pivots off-target.
- **REAR sensor-terminated hop on a 1-board chassis**: LINE → `Fault::REVERSE_UNAVAILABLE` (existing); a **pivot-approaching** REAR seek → `Fault::REVERSE_UNAVAILABLE` (new pivot-scoped check). A **non-pivot** forced-REAR `STOP_AT_LINE` BLIND hop still degrades to dead-reckon reverse (unchanged).
- **Validator rejections** (contradiction, δ>reverse-budget when reachable, REAR-pinned sensor leg on 1 board, reverse-reachable reset-node pivot) → `FieldDataError` at build time naming the escape; `build_routes.py`/CI fails.

## Testing Strategy

**Levels:** Unit (validator pytest, policy parity), Integration (sim↔firmware parity, host C++ FSM), Visual (CTPS eyeball).

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | `resolvePivotWheel` for `(LEFT,REAR)/(RIGHT,REAR)/(*,FRONT)`; twin vs frozen C++ table | Unit (Py) | L↔R flip on REAR; unchanged on FRONT; twin == frozen table for all inputs (AC10) |
| TC2 | REAR pivot in sim, axle landing | Integration | Wheel flipped, δ same, departure forced REAR, axle on `w` within tol (AC1) |
| TC3 | `|θ|>90°` pivot under AUTO, both approaches | Integration (host + sim) | Runs coherently (both legs forced to approach dir); does NOT degrade; final pose on `w` (AC2) |
| TC4 | Binding contradiction (approach FRONT, departure pinned REAR) | Unit + host | Validator rejects; firmware `PIVOT_DIRECTION_MISMATCH` (AC3, AC7c) |
| TC5 | Overshoot budget on a reverse-reachable REAR approach vs a FRONT approach | Integration | REAR offset vs FRONT offset via the one policy call (AC4) |
| TC6 | Authored `direction:"rear"`, δ derived < REAR budget − floor | Unit (validator) | Accepted (AC5) |
| TC7 | Authored `direction:"rear"`, δ in (REAR budget, FRONT budget], reverse-reachable | Unit (validator) | Rejected, names escape (AC5) |
| TC8 | Both legs `direction:"front"`, δ in (REAR, FRONT] band | Unit (validator) | Accepted; runs forward (AC6) |
| TC9 | REAR-pinned sensor leg, `LINE_BOARD_COUNT=1` | Unit (validator) | Rejected (AC7a) |
| TC10 | Reverse-reachable reset-node pivot | Unit (validator) | Rejected unless FRONT-pinned (AC7b) |
| TC11 | REAR seek into pivot, 1 board (host); non-pivot forced-REAR seek, 1 board | Unit + host | Pivot seek faults `REVERSE_UNAVAILABLE`; non-pivot degrades to dead-reckon reverse (AC8) |
| TC12 | `δ > REAR budget` REAR seek reaches runtime | Host | `PIVOT_TRIM_UNDERFLOW` fault |
| TC13 | CTPS reverse-pivot trace | Visual + golden | Arc about flipped-side planted wheel; `direction` + planted wheel in `pivots[]`; travel-frame departure ray (AC9) |
| TC14 | Pivot-free trace `to_dict()` | Unit | Byte-identical, `schema_version==1` (AC9, AC12) |
| TC15 | World-frame geometry: arc-end = `v+δ·û_out`, departure lands on `w`, planted-wheel world-side, both dirs; AUTO-REAR rung asserted fired | Unit | ~1e-6 (AC11) |
| TC16 | Commanded-delta sign vs (wheel, dir) at dispatch | Host | Gross mismatch → tripwire fault; near-180 precluded by the offline `_PIVOT_MAX_TURN_DEG=179` cap |
| TC17 | `PathTracker.goTo` over real field; full pytest + ctest + arduino-cli + routes freshness | Integration | All green; no golden/schema change (AC12) |

**Test data:** the reverse-pivot toy fixture (T9): authored-REAR (δ derived under the REAR budget), **AUTO-REAR** with a `>90°` approach delta (so the AUTO REAR rung genuinely fires — axis-aligned nodes silently resolve FRONT), a `>90°` AUTO pivot for the coherent-run case (TC3), a binding-contradiction triple, a reset-node triple, and a chained reverse-pivot pair. δ values derived from the parsed budgets ± the trim floor, never hardcoded. Mirrored across the Python fixture surfaces (`pivot_field_data.py` + every `SimpleNamespace`/`_FIELD_ATTRS` sibling) and a host C++ routes fixture (resolve nodes **by name**, shadow TU FIRST).
**Board-count override:** host TCs needing 1 board use `target_compile_definitions` (an in-file `#define` is TU-local); Python TCs monkeypatch `sim.runtime.LINE_BOARD_COUNT` before the validator's lazy import (validator.py:540).
**Run:** `uv run --extra dev pytest tools/path_planner/tests/`; `ctest --test-dir build/test`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`; CTPS eyeball over `python -m http.server`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T0 | Config REAR-offset parser + docstring | — | low | `tools/path_planner/validator.py`, `WRO2026/config/ConfigDrive.h` (comment only) | Add `parse_line_sensor_rear_axle_offset_cm()` mirroring the FRONT parser; update the FRONT-only docstring. No behavior change. |
| T1 | Shared `ReversePivotPolicy.h` + Python twin + frozen-table parity test | — | high | `WRO2026/src/nav/ReversePivotPolicy.h`, `tools/path_planner/sim/runtime.py` (policy-twin section only), `tools/path_planner/tests/test_reverse_pivot_policy_parity.py` | Implement `resolvePivotWheel` / `pivotOvershootBudgetCm` + Python twins exactly per Contracts. Derive + comment the L↔R flip from ADR-036. Frozen-C++-table parity test (TC1, AC10). |
| T2 | PathTracker.h: `PivotRef.dir`, `PIVOT_DIRECTION_MISMATCH` | T1 | med | `WRO2026/src/mission/PathTracker.h` | Add `Direction dir` to `PivotRef`; add `Fault::PIVOT_DIRECTION_MISMATCH` + mnemonic. Header only. |
| T3 | PathTracker.cpp: force-SAME, wheel flip, budget, faults, pivot-scoped REAR-seek | T2 | high | `WRO2026/src/mission/PathTracker.cpp` | Replace the force-FRONT block (~2085-2096) with force-SAME: stamp `_approachingPivot.dir`+`_pendingPivot.dir` after `resolveDirection`; at the departure hop force `resolved = _departingPivot.dir` unless a binding override contradicts (→ `PIVOT_DIRECTION_MISMATCH`); dispatch `resolvePivotWheel(_departingPivot.wheel, dir)`. Replace the 3 budget direction-selects with `pivotOvershootBudgetCm(dir)`. Add the pivot-scoped REAR-seek serviceability fault. Add the commanded-delta sign tripwire (TC16). Satisfies AC1-AC4, AC8, plus AC3 runtime side. |
| T4 | Sim: force-SAME, wheel-flip, budget, by-value latch, taken-pivot surface | T1 | high | `tools/path_planner/sim/runtime.py` (FSM section) | Mirror T3 in the sim FSM: by-value pivot latch carrying `dir`, force-SAME on the departure, wheel-flip + `pivot_overshoot_budget_cm`, and expose the taken pivot + resolved direction for CTPS. Keep byte-parity intent with firmware. Satisfies AC1-AC4 (sim side). |
| T7 | Validator: accept REAR, budget branch, contradiction/board/reset/seek rejects | T0, T1 | high | `tools/path_planner/validator.py` | Relax authored-REAR rejection; implement `reverse_reachable` (incl. pipeline-default FRONT pin + seek-classified BLIND legs); per-pinning budget via the T1 twin; reject a binding contradiction, a REAR-pinned sensor leg on <2 boards, and a reverse-reachable reset-node pivot unless FRONT-pinned. Boundary uses the floor test. Satisfies AC5-AC8 (offline). |
| T9 | Reverse-pivot fixtures (Python + host C++ routes) | T1 | med | `tools/path_planner/tests/fixtures/pivot_field_data.py` (+ every sibling `SimpleNamespace`/`_FIELD_ATTRS` surface), host `WRO2026/test/fixtures/*` routes shadow | Author the reverse-pivot toy field (authored-REAR, AUTO-REAR >90°, coherent-run >90° AUTO, binding-contradiction, reset-node, chained). δ derived from parsed budgets ± floor. Resolve host nodes **by name** (shadow FIRST). Update every strict-emitter fixture surface; run FULL pytest (whack-a-mole). |
| T5 | Sim↔firmware parity on a reverse-pivot trace | T3, T4, T9 | med | `tools/path_planner/tests/test_pathtracker_parity.py`, `WRO2026/test/fixtures/field_fixture.py` | Author the reverse-pivot case on a **swept** parity edge (bias `field_fixture`, not a host-only shadow). Regenerate; assert firmware==sim on the reverse pivot (AC10 trace side). |
| T8 | Sim pytest: reverse behavior + world-frame proof | T4, T6, T9 | med | `tools/path_planner/tests/test_runtime_sim.py`, `tools/path_planner/tests/test_reverse_pivot_geometry.py` (new) | Sim cases TC2/TC3/TC5 + the world-frame geometry proof TC15 (arc-end `= v+δ·û_out`, departure on `w`, planted-wheel world-side, both dirs; **assert the AUTO-REAR rung fires**). Blocked by T6 because the arc keyframes live in `track_builder`. Satisfies AC2, AC11. |
| T10 | Host C++ FSM tests | T3, T9 | med | new `WRO2026/test/test_reverse_pivot_*.cpp`, `WRO2026/test/CMakeLists.txt` (orchestrator appends) | Host tests: REAR pivot flips wheel (AC1); >90° AUTO runs coherently (TC3); binding contradiction faults (TC4); pivot REAR-seek 1-board faults + non-pivot degrades (TC11); δ>budget seek fault (TC12); commanded-delta tripwire (TC16). Park mock yaw at edge heading each tick. Satisfies AC1-AC3, AC8. |
| T6 | CTPS reverse-pivot arc render | T1, T4, T9 | high | `tools/path_planner/ctps/track_builder.py`, `ctps/config.py`, `ctps/assets/renderer.js`, `ctps/player_html.py`, `tools/path_planner/tests/test_ctps_*` | Consume the sim's taken-pivot direction; `pivots[]` gains `"direction"` + the **planted (post-flip)** wheel (inside the non-empty-gated list; `schema_version` stays 1). Flip the planted-wheel side + pin for REAR (arc formula reused; `h0` carries +180); fix the `pending_pivot_land` travel-frame departure ray. **Recorder/`trace_loader` untouched** (direction derived). Golden TC13 + byte-identical TC14; eyeball over localhost. Satisfies AC9. |
| T11 | Full-suite green + no-golden-change gate | T5, T6, T7, T8, T10 | low | (verification only) | FULL `uv run --extra dev pytest`, FULL `ctest`, `arduino-cli compile`, `build_routes.py` + `git diff --exit-code` on `routes.h/.cpp` + emitter goldens. Prove AC12. |
| T12 | Docs: ADR-071 + ADR-036/044/054 updates + Roadmap + DECISIONS + close #54 | T3, T7 | low | `docs/decisions/071-reverse-pivots.md` (new), `docs/decisions/036,044,054`, `Roadmap.md`, `DECISIONS.md` | ADR-071 (reverse-pivot rule, wheel-flip derivation, force-SAME + `PIVOT_DIRECTION_MISMATCH`, conservative budget, pivot-scoped seek gap, reset-node fence, arc-end geometry). Update the FRONT-only "trigger to revisit" clauses in ADR-036/044/054 to "Implemented by ADR-071 (#54)". Add the B3.8-reverse gate + `PIVOT_SCRUB_FACTOR` note to Roadmap §13 + Risk #9. Bump `DECISIONS.md`. Reference #54 as closed on merge. |

**Parallelism:** T0, T1 start immediately. T2→T3 is the firmware spine; T4 (sim FSM) needs T1; T7 (validator) needs T0+T1. T9 (fixtures) needs T1 and gates the test tasks. T6 (CTPS) needs T1+T4+T9 (the sim must expose direction and stop forcing FRONT before a REAR pivot appears in a trace). T8 needs T4+T6+T9. T5/T8/T10 parallelize once blockers resolve; T11 is the final gate; T12 needs firmware + validator settled. **CMakeLists.txt appends (T9, T10) are serialized by the execute orchestrator**, not the code-writers.

## Notes for Implementer

- **Force-SAME is the load-bearing change.** Force-FRONT was the direction-coherence mechanism; force-SAME generalizes it (force departure = approach dir) rather than deleting it. There is **no** "FRONT-unavailable → skip pivot" precedent — that path faults (PathTracker.cpp:2085-2096). Under AUTO, every `|θ|>90°` pivot resolves mixed and would degrade under a naive skip; force-SAME makes them run.
- **The wheel flip is the crux — derive it, don't guess.** Restate ADR-036's four-step chain with the reverse-roll step (driven wheel rolls backward ⇒ opposite braked wheel for the same body θ). The world-frame proof (T8/TC15) is what proves the three surfaces are *right*, not merely mutually consistent. **Do NOT derive the wheel from the live commanded delta** (reintroduces ADR-036 Blocker-1); the table is the wheel authority, the delta is only a tripwire.
- **Arc-end is `v + δ·û_out(travel)`, not on `v`.** The departing trim nets it to `w`. Any test asserting "arc-end on `v`" codifies wrong geometry (ADR-054).
- **Latch-ordering trap:** `_pendingPivot = _approachingPivot` is copied before the hop's direction is resolved, so stamp `dir` on **both** copies after `resolveDirection`, or the departing copy stays `AUTO` and force-SAME misfires.
- **Sim latch must be a by-value copy** (not a mutable ref into `_pivot_records`) — mutating a shared record corrupts the table across traversals.
- **Do NOT widen `reverseUnavailable`.** A blanket REAR-seek fault regresses the documented non-pivot forced-REAR `STOP_AT_LINE`→dead-reckon-reverse degrade (runtime.py:483-494). Scope the new fault to `_approachingPivot.active` seeks.
- **Symbolic budgets everywhere.** 7.68 is an unmeasured placeholder; reference `Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM` / the parsed value and derive test δ from it, so a bench retune reddens only the four known drive-constant surfaces.
- **Axis-aligned fixtures hide the AUTO-REAR case.** The AUTO REAR rung fires only for `|body delta| > BIDIRECTIONAL_TURN_THRESHOLD_DEG` (90°, strict `>`, ties→FRONT). The AUTO-REAR triple MUST use a `>90°` approach delta and the test MUST assert the rung fired (false-green trap).
- **Strict-emitter fixture fanout:** T9 adds attributes the emitter reads; grep every `SimpleNamespace(`/`_FIELD_ATTRS`/`make_*field` surface and run the FULL pytest (2-3 whack-a-mole rounds).
- **CTPS byte-identity survives by construction:** keep any new key inside the non-empty-gated `pivots[]`, never bump `schema_version`, never touch non-pivot keyframe emission; recorder/`trace_loader` untouched. Real field has empty `PIVOT_TURNS` → blast radius is fixtures only.
- **Rollback:** additive + gated by force-SAME. If a reverse pivot misbehaves on the bench, re-pin the offending pivot's legs `direction:"front"` in `field_data.json` (no code change) to restore forward-only behavior for that turn.
