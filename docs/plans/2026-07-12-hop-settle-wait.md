# Hop Settle Wait Plan

**Goal:** After the chassis brakes to rest at a node, hold the PathTracker FSM idle (brakes engaged, non-blocking) for a configurable `Config::HOP_SETTLE_MS` before the next maneuver, so residual inertia dissipates before the re-anchor turn/drive — and reflect that pause in CTPS playback. Closes issue #122.

**Approach:** Add an explicit `State::SETTLE` to the PathTracker FSM (a `millis()`-gated non-blocking dwell, iron-rule-1 compliant). Each *braked* arrival transition routes through SETTLE before planning the next hop; a cold pre-edge turn routes through SETTLE again before the move (the turn→move dwell). Non-braking arrivals (pass-throughs, blind-chain intermediates), the goal's first plan, arrival at a RESET node, and the return-to-node prologue skip it; `HOP_SETTLE_MS == 0` disables it entirely (byte-identical dispatch to today). The settle emits **no parity record and moves no pose**, so the CI parity job is unaffected (ADR-050 excludes timing by construction). CTPS shows the pause **at the render layer only** (`track_builder.py`), which already classifies each hop `braked` vs pass-through (ADR-059 run-grouper) and already owns synthetic pauses (`append_dwell_keyframe`, `MANIP_NOMINAL_S`); `HOP_SETTLE_MS` is parsed from Config via `config_parse` (never a hand-mirrored literal). The parity-locked `PathTrackerSim` is untouched.

## Scope

- **In scope:**
  - `Config::HOP_SETTLE_MS` in `WRO2026/config/ConfigNav.h` (default 100 ms, compile-time override hook, ConfigChecks tripwire).
  - `PathTrackerClass::State::SETTLE` + a non-blocking `millis()` dwell gate with a resume target (plan / edge-after-turn).
  - Settle fires at three boundaries on **normal** hops: post-stop before the next maneuver, turn→move after a cold turn, and goal arrival (before IDLE/DEQUEUE).
  - Parity dump driver handles `State::SETTLE` (advance mock clock); host ctest coverage of the settle-fire matrix.
  - CTPS render-layer dwell in `track_builder.py` at the matching seams; `HOP_SETTLE_MS` parsed into `ProfileConfig`; a pytest asserting the render half of the matrix; regen CTPS goldens.
  - ADR-061 recording the behavior + the render-not-sim decision + the eligibility tether.

- **Out of scope (deferred / separate):**
  - Settle inside the **wall-zero reset interlude** (`WALL_STOP → REANCHOR → WALL_DEPART`) and the **return-to-node prologue** — deferred (file a follow-up if wanted). Arrival *at* a RESET node also skips settle (the wall interlude owns that boundary).
  - A **per-edge** `HOP_SETTLE` override — v1 ships only the global constant (per the issue). Per-edge tuning belongs to the #104 family (per-edge EDGES fields).
  - The **spatial** stop-early margin (#104) — orthogonal; composes but is a separate issue.
  - Modeling the dwell in `PathTrackerSim` / the parity trace (rejected — see Decisions).
  - Any change to `tShape` / `stopAtLine` braking behavior itself.

## Decisions

- **Explicit `State::SETTLE`, not an inline gate** — matches the codebase's per-state FSM discipline, is observable via `state()`/EventTrace, and is the natural home for the parity driver + host harness per-state helpers. An inline `millis()` hold in `PLAN_NEXT_HOP`/`TURNING` would need a re-entry latch to avoid re-firing the arrival hooks and hides the dwell from `state()`. (Phase 4 pick.)
- **CTPS: render-layer, not sim-modeled** — Fable ENDORSE (veto authority). `ctps_dump` uses a **mock** PathTracker (`ctps_mocks/PathTracker.h`) that records one `goto` and reports instant-idle, so the real FSM never runs during the dump; the dwell can only originate downstream. The render layer already owns synthetic time (`append_dwell_keyframe`, `MANIP_NOMINAL_S`) and already classifies `braked` vs pass-through. Sim-modeling would touch the parity-locked sim + normalizer + parity tests, trip ADR-050's named "sim must model timing" trigger-to-revisit for a pause parity can't see, and still wouldn't unify the eligibility logic (the firmware copy stays in C++ regardless). (Phase 4.5.)
- **`HOP_SETTLE_MS` parsed from Config in Python, never hand-mirrored** — `profile.py` sources every timing constant through `config_parse` (issue #44 "no stale-literal"; ADR-058). A hand-copied literal would silently render the old pause after a firmware retune. (Fable refinement.)
- **Proportionate eligibility tether, not a policy header** — one host ctest (firmware SETTLE fires exactly on the matrix) + one pytest (render dwell fires on the same matrix), both citing the single **Settle-fire matrix** below. A full header-only `SettlePolicy.h` + CSV truth-table (à la `EdgeDispatchPolicy`) is disproportionate: the eligibility rules live on FSM *transitions*, not resolved per-hop inputs. Graduate to a policy header only if the deferred wall-reset/return settles land with their own conditions. (Fable refinement.)
- **`HOP_SETTLE_MS == 0` fully disables** — the FSM never enters SETTLE and CTPS renders no dwell keyframes; dispatched primitive sequence and render are byte-identical to pre-feature. This is the single rollback knob.
- **Default 100 ms** (issue suggestion); a stop-and-turn hop adds ~2× (pre-turn + turn→move). Bench-tune later.
- **"Pause only" in CTPS** — no timeline marker/lane (user choice); a dwell keyframe advances the clock with the pose held.
- **Codex approach consult skipped** — Codex CLI/model unsupported (`gpt-5.6-sol` requires a newer Codex); run `/codex:setup` to enable. Proceeded on the Fable consult alone.

## Acceptance Criteria

- [ ] AC1 — `Config::HOP_SETTLE_MS` exists in `ConfigNav.h`, default `100`, guarded by `#ifndef HOP_SETTLE_MS_DEFAULT`; `ConfigChecks.h` asserts a sane ceiling. `-DHOP_SETTLE_MS_DEFAULT=0` compiles and disables the feature.
- [ ] AC2 — On a braked stopped hop (`LINE_OVERSHOOT` non-pass, `STOP_AT_LINE`, `PROJECTION_FOLLOW` stop, non-chain `BLIND_FORWARD`) with `HOP_SETTLE_MS>0`, the FSM enters `State::SETTLE` and holds ≈`HOP_SETTLE_MS` (non-blocking, Drive idle) before the pre-edge turn (or the move when already aligned).
- [ ] AC3 — After a **cold** pre-edge turn on a normal hop, the FSM settles again ≈`HOP_SETTLE_MS` before dispatching the move (turn→move dwell). The return-to-node prologue's pre-edge turn does **not** settle.
- [ ] AC4 — On the FINAL braked hop of a goal, the FSM settles before returning to `IDLE`/`DEQUEUE` (goal-arrival dwell) so a following Mission step sees a settled chassis.
- [ ] AC5 — Settle is SKIPPED (SETTLE never entered) on: LINE/projection pass-throughs, ADR-059 blind-chain intermediate hops, a goal's first plan, arrival at a `RESET` node, the return-to-node prologue, and `HOP_SETTLE_MS==0`. In every skip case the dispatched primitive sequence is byte-identical to today.
- [ ] AC6 — The settle emits no parity record and moves no pose; the CI `parity` job stays green at the shipped default (100). The parity dump driver advances the mock clock in `State::SETTLE` so the dump never stalls.
- [ ] AC7 — CTPS playback holds ≈`HOP_SETTLE_MS` at each braked hop (pre-maneuver), after each cold turn (turn→move), and at goal arrival — and holds **not** at pass-throughs / chain-intermediates / wall-return interludes — matching the firmware matrix. `HOP_SETTLE_MS` is parsed from Config; `==0` renders no dwell.
- [ ] AC8 — A paired eligibility tether exists: a host ctest asserts firmware SETTLE fires exactly on the Settle-fire matrix; a pytest asserts the CTPS render dwell fires on the same matrix.
- [ ] AC9 — Host ctest suite, Python pytest suite (incl. parity + CTPS goldens), and `arduino-cli` compile all pass; regenerated CTPS goldens are committed.

## Data Model

Firmware (`PathTracker.h`):

```cpp
enum class State : uint8_t {
    IDLE, DEQUEUE, RETURN_TO_NODE, PLAN_NEXT_HOP, TURNING,
    LINE_FOLLOW, LINE_OVERSHOOT, PROJECTION_FOLLOW, BLIND_FORWARD,
    STOP_AT_LINE, WALL_STOP, REANCHOR, WALL_DEPART,
    SETTLE,        // NEW — non-blocking post-stop dwell
    FAULT
};

// Where to go when the dwell elapses.
enum class SettleResume : uint8_t {
    PLAN,             // -> State::PLAN_NEXT_HOP (post-stop, pre-maneuver / goal arrival)
    EDGE_AFTER_TURN   // -> _dispatchEdgeAfterTurn() (post-cold-turn, pre-move)
};

uint32_t     _settleDeadlineMs = 0;                 // millis() + HOP_SETTLE_MS, stamped on entry
SettleResume _settleResume     = SettleResume::PLAN;
```

Config (`ConfigNav.h`):

```cpp
#ifndef HOP_SETTLE_MS_DEFAULT
#define HOP_SETTLE_MS_DEFAULT 100
#endif
/// Settle dwell (ms) held after a stopped hop before the next maneuver so residual
/// inertia dissipates before a turn/drive. 0 disables (SETTLE never entered). Skipped on
/// a pass-through hop, a blind-chain intermediate, the first plan, a RESET-node arrival,
/// and the return/wall interludes. See ADR-061.
constexpr uint32_t HOP_SETTLE_MS = HOP_SETTLE_MS_DEFAULT;   // TODO bench-tune
```

Python (`ctps/profile.py` → `ProfileConfig`): a new `hop_settle_ms: int` field, populated via `config_parse.parse_config_int("HOP_SETTLE_MS", ...)` (mirrors the existing `TSHAPE_START_SPEED` etc. pattern — not a literal).

## Contracts & Interfaces

Single source of truth for the cross-task seams.

### Settle-fire matrix (the shared contract — both the host ctest and the pytest assert THIS)

| Arrival / boundary | Firmware signal | Braked? | Settle fires? |
|---|---|---|---|
| `LINE_OVERSHOOT`, normal | `!_passThrough` | yes | **yes** |
| `LINE_OVERSHOOT`, pass-through | `_passThrough` | no | no |
| `PROJECTION_FOLLOW`, stop-at-B | `!_projPassThrough` | yes | **yes** |
| `PROJECTION_FOLLOW`, pass-through | `_projPassThrough` | no | no |
| `STOP_AT_LINE` | always | yes | **yes** |
| `BLIND_FORWARD`, non-chain | `!_chain.active` | yes | **yes** |
| `BLIND_FORWARD`, chain intermediate | `_chain.active` | no | no |
| Cold pre-edge turn done (normal hop) | `TURNING` idle, `!_returnActive` | n/a | **yes** (turn→move) |
| Return-prologue pre-edge turn | `_returnActive` | n/a | no |
| Goal arrival (final braked hop) | braked arrival, `cur==dest` | yes | **yes** |
| Goal first plan (standing start) | `_planIsFirstAfterDequeue` | n/a | no |
| Arrival at a `RESET` node | `getNodeType(arrived)==RESET` | yes | no (wall interlude owns it) |
| Any, `HOP_SETTLE_MS==0` | — | — | no |

Every "yes" additionally requires `Config::HOP_SETTLE_MS > 0`.

### Firmware signatures (owner: T2)

- `State::SETTLE`, `SettleResume`, `_settleDeadlineMs`, `_settleResume` — added to `PathTrackerClass`.
- `void _enterSettle(SettleResume resume)` — stamps `_settleDeadlineMs = millis() + Config::HOP_SETTLE_MS`, sets `_settleResume`, `_state = State::SETTLE`. Called only where the matrix says "yes".
- SETTLE handler: rollover-safe `if ((int32_t)(millis() - _settleDeadlineMs) >= 0)` → `PLAN_NEXT_HOP` (PLAN) or `_dispatchEdgeAfterTurn()` (EDGE_AFTER_TURN).
- Reset `_settleDeadlineMs`/`_settleResume` on `begin`/`abort`/`_enterFault` (defensive; the state itself is the guard).

### `state()` consumers that MUST add `case State::SETTLE` (avoid `-Wswitch`)

- `WRO2026/src/mission/PathTracker.cpp` — `_stepFsm()` switch (owner: T2).
- `WRO2026/test/test_path_tracker_parity_dump.cpp` — driver switch; add `case State::SETTLE: driveSettle();` that advances the mock clock past the deadline (owner: T3).
- (The `ctps_mocks/PathTracker.h` mock does **not** switch on `State` — no change.)

### CTPS render seams (owner: T6, in `track_builder.py`)

- After a settle-eligible braked `flush_run()` — insert `append_dwell_keyframe(hop_settle_s)`; **suppress** when the braked tag ∈ `_WALL_RESET_TAGS` / `_RETURN_DRIVE_TAGS`.
- After a rendered cold turn (`turn_mag > NAV_TURN_TOLERANCE_DEG`) when the next primitive is a drive — insert the dwell (deferred one-primitive flag); **suppress** when the next tag ∈ `_RETURN_DRIVE_TAGS` (return prologue turn). Same seam on the pivot-arc path.
- After the goal-arrival strict `flush_run(strict=True)`.
- All gated on `hop_settle_s > 0`.

### Naming (must match across tasks)

- Config symbol: `HOP_SETTLE_MS` (C++) ↔ `parse_config_int("HOP_SETTLE_MS", ...)` (Python) ↔ `ProfileConfig.hop_settle_ms`.
- FSM enum member: `State::SETTLE`.

### File ownership

| File | Owner | Consumers |
|------|-------|-----------|
| `WRO2026/config/ConfigNav.h`, `config/ConfigChecks.h` | T1 | T2, T5 |
| `WRO2026/src/mission/PathTracker.h`, `PathTracker.cpp` | T2 | T3, T4 |
| `WRO2026/test/test_path_tracker_parity_dump.cpp` | T3 | — |
| `WRO2026/test/test_path_tracker_harness.h`, new `test_path_tracker_settle.cpp`, `WRO2026/test/CMakeLists.txt` | T4 | — |
| `tools/path_planner/ctps/profile.py` (+ `config_parse.py` if a helper is missing) | T5 | T6 |
| `tools/path_planner/ctps/track_builder.py` | T6 | T7 |
| `tools/path_planner/ctps/tests/*` (goldens + new `test_ctps_hop_settle.py`) | T7 | — |
| `docs/decisions/061-hop-settle-wait.md`, `DECISIONS.md` | T8 | — |

## Error Handling

- **Dwell never hangs** — non-blocking `millis()` gate; `update()` returns each tick, so the main loop keeps ticking every module. On hardware `millis()` advances naturally; in the parity dump the driver advances the mock clock (T3); in host tests `driveSettle()` advances `g_mockMillis` (T4).
- **Rollover-safe compare** — use `(int32_t)(millis() - _settleDeadlineMs) >= 0`, never `millis() >= deadline` (matches the repo's `_drainBlocking` deadline idiom).
- **Drive fault during SETTLE** — Drive is idle/braked in SETTLE; the top-of-`_stepFsm()` fault guard is inert (no Drive fault while parked). No special exemption needed.
- **`abort()` / `_enterFault()` from SETTLE** — `abort()` already stops the chassis, clears the queue, and resets `_state`; ensure the settle members are reset there so a later goal can't inherit a stale deadline.
- **CTPS with `HOP_SETTLE_MS==0`** — the parse yields 0; the `> 0` guard appends no dwell keyframe; goldens are identical to pre-feature.
- **Firmware↔render drift** — guarded by the paired eligibility tether (T4 + T7) against the Settle-fire matrix; a mismatch reddens CI. If the firmware settle ever emits a trace record by mistake, the strict parity loader raises loudly (`parity.py:448`) rather than drifting silently.

## Testing Strategy

**Levels:** Unit (host ctest + pytest), Integration (parity job, CTPS golden render), Compile (`arduino-cli`).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | Braked stopped hop, `HOP_SETTLE_MS=100` | Host ctest | FSM enters `SETTLE`, holds until mock clock passes deadline, then dispatches the pre-edge turn/move |
| TC2 | Pass-through hop (`_passThrough`) | Host ctest | FSM never enters `SETTLE`; transition byte-identical to today |
| TC3 | Blind-chain intermediate (`_chain.active`) | Host ctest | No `SETTLE`; final chained (braked) hop DOES settle |
| TC4 | Cold turn then move (normal hop) | Host ctest | Two settles: one pre-turn, one turn→move; return-prologue turn settles neither |
| TC5 | Goal arrival (final braked hop) | Host ctest | `SETTLE` before `IDLE`/`DEQUEUE`; first plan of a fresh goal does NOT settle |
| TC6 | Arrival at a `RESET` node | Host ctest | No settle; wall interlude entered as today |
| TC7 | `-DHOP_SETTLE_MS_DEFAULT=0` | Host ctest | `SETTLE` never entered; full goal dispatch byte-identical to pre-feature |
| TC8 | Parity dump at default (100) | pytest `parity` | Trace unchanged (no settle record, pose unchanged); parity green; dump completes (driver advances clock) |
| TC9 | CTPS render, braked hop + cold turn + goal arrival | pytest | Dwell keyframes at all three matrix seams; total duration increases by the expected `HOP_SETTLE_MS` per seam |
| TC10 | CTPS render, pass-through / chain-intermediate / wall-return | pytest | No dwell keyframe at those boundaries |
| TC11 | CTPS render, `HOP_SETTLE_MS=0` | pytest | No dwell keyframes; render identical to pre-feature goldens |
| TC12 | ConfigChecks ceiling | Compile | An over-ceiling `HOP_SETTLE_MS` fails the static_assert with a clear message |

**Test data:** existing `routes_fixture` / toy field for the host FSM tests (mock clock via `mockAdvanceMillis`); existing CTPS trace fixtures for the render pytest; regen the affected CTPS duration goldens.
**Run commands:** `ctest --test-dir build/test --output-on-failure`; `uv run --extra dev pytest tools/path_planner/tests/`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Config constant + tripwire | — | low | `WRO2026/config/ConfigNav.h`, `WRO2026/config/ConfigChecks.h` | Add `HOP_SETTLE_MS` (default 100) with `#ifndef HOP_SETTLE_MS_DEFAULT` hook per the file idiom; add a ConfigChecks static_assert for a sane ceiling (e.g. `<= 2000`) with a clear message. Satisfies AC1, AC12/TC12. |
| T2 | `State::SETTLE` FSM gate | T1 | high | `WRO2026/src/mission/PathTracker.h`, `WRO2026/src/mission/PathTracker.cpp` | Add `State::SETTLE`, `SettleResume`, `_settleDeadlineMs`, `_settleResume`, `_enterSettle(...)`, and the SETTLE case in `_stepFsm()`. Wrap the four braked arrival transitions (`LINE_OVERSHOOT !_passThrough`, `STOP_AT_LINE`, `PROJECTION_FOLLOW` stop, `BLIND_FORWARD !_chain.active`) with `_enterSettle(PLAN)` when `HOP_SETTLE_MS>0 && getNodeType(arrived)!=RESET`; else transition as today. Wrap `TURNING`-cold-turn-done (`!_returnActive`) with `_enterSettle(EDGE_AFTER_TURN)`. Reset settle members on begin/abort/fault. Rollover-safe compare. Follow the exact Settle-fire matrix. Satisfies AC2–AC5. |
| T3 | Parity dump driver handles SETTLE | T2 | med | `WRO2026/test/test_path_tracker_parity_dump.cpp` | Add `case State::SETTLE: driveSettle();` to the driver switch — advance `g_mockMillis` past `_settleDeadlineMs` and re-tick until the FSM leaves SETTLE, so the dump completes. Confirm no new parity record and parity stays green. Satisfies AC6/TC8. |
| T4 | Firmware host tests + tether | T2 | med | `WRO2026/test/test_path_tracker_harness.h`, new `WRO2026/test/test_path_tracker_settle.cpp`, `WRO2026/test/CMakeLists.txt` | Add a `driveSettle()` harness helper (advance mock clock past the deadline). New ctest covering TC1–TC7 (the firmware half of the eligibility tether) asserting SETTLE fires exactly on the Settle-fire matrix and dispatch is byte-identical in every skip case. Register the target in CMakeLists. Satisfies AC2–AC5, AC8 (firmware half), AC9. |
| T5 | CTPS profile config field | T1 | low | `tools/path_planner/ctps/profile.py` (+ `tools/path_planner/config_parse.py` only if a parse helper is missing) | Add `hop_settle_ms` to `ProfileConfig`, populated via `config_parse.parse_config_int("HOP_SETTLE_MS", ...)` — NOT a hand-mirrored literal. Satisfies AC7 (config-sourced). |
| T6 | CTPS render dwell | T5 | med | `tools/path_planner/ctps/track_builder.py` | Insert `append_dwell_keyframe(hop_settle_s)` at the three seams in the Contracts (braked flush with wall/return suppression; post-cold-turn with return-prologue suppression + pivot-arc seam; goal-arrival strict flush), gated on `hop_settle_s>0`. Match the Settle-fire matrix. Satisfies AC7. |
| T7 | CTPS pytest + goldens | T6 | low | `tools/path_planner/ctps/tests/*` (regen affected goldens + new `test_ctps_hop_settle.py`) | Regenerate the affected CTPS duration goldens (durations shift after each dwell — expected). New pytest covering TC9–TC11 (the render half of the eligibility tether) asserting the dwell fires on the same matrix and `==0` matches pre-feature. Satisfies AC7, AC8 (render half), AC9. |
| T8 | ADR-061 + index | T2, T6 | low | `docs/decisions/061-hop-settle-wait.md`, `DECISIONS.md` | Write ADR-061 (rule = the Settle-fire matrix + non-blocking SETTLE state; why = inertia-corrupts-reanchor; how = firmware gate + render-owned CTPS pause parsed from Config; trigger = deferred wall-reset/return settles or a per-edge override). Add the index line + bump the status-summary count in `DECISIONS.md`. |

Parallelism: T1 first. Then T2 ∥ T5. Then T3 ∥ T4 (after T2) and T6 (after T5). Then T7 (after T6) and T8 (after T2+T6).

## Notes for Implementer

- **RAII builder rule** does not apply here (SETTLE issues no Drive builder — it just holds), but if any Drive call is ever added, keep it a single discarded-temporary `.async()` statement.
- **Do not exempt SETTLE from the top-of-`_stepFsm()` Drive-fault guard** — Drive is parked (idle, no fault) during the dwell, so the guard is naturally inert; adding an exemption would be dead code.
- **`setNavActive` during SETTLE** — `update()`'s `Localization.setNavActive(_state != IDLE && _state != FAULT)` keeps nav-active TRUE during SETTLE, so the operator-offset integrator does NOT accumulate phantom offset while parked. This is correct; do not special-case it.
- **The braked decision is local to each arrival state** — do not introduce a persistent `_hopBraked` member that can go stale; each arrival state already knows its brake status from the live flag (`_passThrough` / `_projPassThrough` / `_chain.active`) at the transition point.
- **Firmware and render duplicate the eligibility mapping by necessity** (the firmware copy is in the C++ FSM, the render copy in `track_builder`); the paired tether (T4 + T7) against the one Settle-fire matrix is the guard. Keep both tests citing this spec's matrix so a future edit updates both.
- **CTPS golden churn is expected** — every keyframe after an inserted dwell shifts in time; regenerate and commit. This is test maintenance, not a parity regression.
- **CI freshness** — regenerated CTPS goldens must be committed or the pytest job reddens; no `routes.h/.cpp` change is involved (no field edit).
- **Before opening the PR** (execute phase): read `CONTRIBUTING.md`; the PR closes #122.
- **Latest ADR number is 060** — confirm 061 is unclaimed at write time (guard against a parallel-branch collision).
