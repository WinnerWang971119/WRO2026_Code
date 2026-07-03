# Line-Arrival Heading Reset (Wall-Free) Plan

**Goal:** Bound IMU-yaw drift over a long run by re-zeroing the robot's heading belief for free whenever it line-follows straight through a collinear pass-through node — no wall, no maneuver — binding to the incoming LINE edge's already-known `heading_deg`. Closes issue #73.

**Approach:** Extract the heading-bind step out of `LocalizationClass::reanchorAtWall` into a shared private core, exposed as a new heading-only `reanchorHeading(headingDeg)`. PathTracker calls it inline at its single arrival chokepoint (`_onPlanNextHop`) when the just-completed arrival was a collinear pass-through, gated by two independent global config flags (projection pass-throughs default ON, physical-junction pass-throughs default OFF) and a strict sanity gate. It is instantaneous (one IMU read + offset math), heading-only, no motion, no fault — a rejected sample or failed read soft-skips and the run continues. No `field_data` / `routes` / `ResetNode` changes: projection-node placement is the opt-in.

## Scope

- **In scope:**
  - `LocalizationClass::reanchorHeading(float headingDeg, float* outPreBindDeltaDeg = nullptr) -> bool` — heading-only re-anchor: guard `_navActive`, read IMU once, strict-gate, re-bind `_yawOffsetDeg`, refresh the heading cache/stamp. Returns `false` (no mutation) on `!_navActive`, `!_zeroed`, a failed IMU read, or a strict-gate rejection.
  - A private `_bindHeadingFromYaw(float imuYaw, float headingDeg)` core shared by `reanchorHeading` and `reanchorAtWall` (single source of truth for the offset-bind + cache refresh). `reanchorAtWall` is refactored to call it; **its observable behavior is unchanged**.
  - PathTracker `_maybeHeadingReset(uint8_t cur, bool firstPlan, float incomingBodyHeadingDeg)` fired at `_onPlanNextHop`, on a collinear pass-through only, with a node-type switch that enumerates all four `NodeType` values (PROJECTION → projection knob, LINE → junction knob, BLIND/RESET → no-op) — no silent `default`, so a future enum member trips `-Wswitch`.
  - Four new `Config` knobs in `config/ConfigNav.h` (each with the `_DEFAULT` compile-time override hook): `LINE_HEADING_RESET_ON_PROJECTION` (default **true**), `LINE_HEADING_RESET_ON_JUNCTION` (default **false**), `LINE_HEADING_RESET_STRICT_ENABLED` (default **true**), `LINE_HEADING_RESET_TOLERANCE_DEG` (default **10.0f**).
  - Loud observability: a Serial log on every fire and every skip, plus a single `EventTrace.pushEvent(...)` on a fire — called directly from `_maybeHeadingReset`, exactly as the wall interlude already calls `EventTrace.pushEvent` (`PathTracker.cpp:1156/1185`). No new EventTrace accessors and no EventTrace edit.
  - Sim parity: mirror the fire decision in `PathTrackerSim` and emit a belief-only `heading_reset` **sim-only primitive tag** modeled on `ACT_REANCHOR` — the full three-place parity contract (see Contracts). Emitted **only on a successful bind**.
  - Host C++ tests for `reanchorHeading` (accept / strict-reject / failed-read / `!_navActive` / `!_zeroed` guard / `_offset` untouched / cache refreshed) and for the PathTracker gating logic (incl. the REAR body-corrected heading and the forced-state RESET/BLIND no-op).
  - Python sim tests for the `heading_reset` record + accept-path decision parity.
  - ADR-052 recording this as a heading-only re-anchor surface.
- **Out of scope:**
  - Any change to `field_data.json` / `field_data.py` / production `routes.h` / `routes.cpp` / `ResetNode` / `RESET_NODES` — there is none. Projection-node placement (an existing authored node property, ADR-046) is the opt-in. (Test-fixture routes under `WRO2026/test/` are a separate surface — see T6.)
  - Any Drive motion, along-line position re-anchor, or wall maneuver — heading-only, no motion.
  - Any behavior change to the existing wall reset (`reanchorAtWall` / `RESET` node interlude).
  - Cardinal-only restriction — the bind is direction-agnostic and binds to the exact authored `heading_deg`.
  - On-hardware tuning of the tolerance — ships as a documented default (10°) to be bench-calibrated later.
  - A settledness sample-gate and always-on (feature-off) drift telemetry — deferred to the ADR's trigger-to-revisit (see Decisions).
  - CTPS integration — a heading reset is not a `drive.method`; the CTPS 3-place contract is untouched.

## Decisions

- **Shared-bind structure — Approach A via a PRIVATE core, not public-calls-public.** (User delegated this fork to the Fable consult, which chose A.) `reanchorAtWall` and `reanchorHeading` each do their *own* single IMU read and their *own* gate, then both call `_bindHeadingFromYaw`. `reanchorAtWall` must NOT call the public `reanchorHeading` — that would either double-read the IMU (violating its documented "one sample backs both the gate and the bind" contract, `Localization.cpp:205-210`) or apply the wrong gate/failure semantics (wall REJECT faults; line REJECT soft-skips). Duplicating the bind math (fork B) was rejected: two copies of the invariant the single-writer class exists to protect will drift.
- **Fire at the `_onPlanNextHop` chokepoint**, right after the wall `_maybeEnterReset` block and before the destination check — the one place every LINE arrival (stop-and-go *and* pass-through) funnels through. Structurally symmetric with the wall hook. Sampling one tick later than the in-state alternative is physically irrelevant (~30 ms / ~1 cm, heading unchanged), and firing before the next hop's direction resolution means the next hop is planned against the freshly-corrected heading.
- **Node-type switch enumerates all four `NodeType`s; BLIND/RESET → no-op is DEFENSE-IN-DEPTH, not a live-bug guard.** Per ADR-048 (Implemented, validator-enforced via `_reject_line_follow_into_blind`) no LINE edge may be incident to a `reset` node, and `_passThrough` is latched only on a LINE-edge arrival (`PathTracker.cpp:907/1064`) and consumed at every dispatch (`:2127/:2162/:2221/:2325`) — so on any validator-passed field a RESET node is reached only over a BLIND edge with `_passThrough == false`. The BLIND/RESET no-op branch therefore guards two off-nominal cases only: ADR-048's own trigger-to-revisit ("a reset node ever gains a legitimate line-follow approach") and hand-edited routes tables that bypass the validator. Enumerating all four cases (no silent `default`) means a future `NodeType` member trips `-Wswitch`.
- **`!firstPlan` gate** — mirrors the wall hook's gate. `_passThrough` provably cannot be true on a first plan today; the gate documents the invariant instead of trusting five distant consume/clear sites forever.
- **Two independent global flags, no per-node authoring.** Projection pass-throughs (clean heading — the line-follow PID never freezes) default ON; physical-junction pass-throughs (junction PID-freeze can inject a small wiggle) default OFF. Shared strict-enable + one tolerance.
- **Pass the incoming heading EXPLICITLY, don't rely on member ordering.** `_maybeHeadingReset` takes `incomingBodyHeadingDeg` as a parameter, captured from `_resolvedBodyHeading` at the top of `_onPlanNextHop` before any dispatch can overwrite it (`_resolvedBodyHeading = bodyTarget` at `:1757`). This removes the ordering fragility a comment could only document.
- **Reject on `!_navActive`, don't just warn.** `reanchorHeading` returns `false` with no mutation when `_navActive == false`. The reset is only ever reached from the nav hot path (`setNavActive(_state != IDLE && _state != FAULT)`, `PathTracker.cpp:648`), so the integrate gate is closed and the closed→open edge re-baselines from a live read in the new frame — but rebinding the world frame while the gate is OPEN (idle) would deposit the discontinuity into `thetaDeg` on the next `update()` (`gotcha-localization-rezero-reopen-edge`). Rejecting makes that invariant enforced, not merely asserted. `reanchorHeading` never touches `_offset` / the integrator baselines.
- **Strict gate is a gross-wrong-state detector, not a wiggle/drift discriminator.** At ~10° it cannot separate line-follow wiggle (±1–3°) from IMU drift (±1–3°/min) — it rejects perpendicular/kidnapped/stale-flag cases (~90°/180°), and (by design) passes a *parallel* wrong line where the bind is coincidentally still correct. Documented as such; default ON.
- **Soft-skip, loud — but the parity record and the Serial log are SEPARATE channels.** A failed IMU read or a strict-gate rejection returns `false` with no mutation and the run continues (no fault). The **parity trace record fires only on a successful bind** (never on skip/reject), so the driftless sim — which has no reject path — and the firmware match on the accept path. The **Serial log** is a separate human-readable channel: it logs a line on a fire (with the pre-bind delta) and on a *skip* — where "skip" means an IMU-read failure or a strict-reject **with the relevant knob ON** (read-failure logs `delta=n/a` since the out-param is untouched). Knob-OFF (returns before any read) and the BLIND/RESET defensive branch are **silent** and are not "skips". This is what keeps "off means off, no cost", surfaces drift when the gate rejects (the reset was most needed), and gives free per-node drift data on the enabled paths.
- **Telemetry only when the relevant knob is ON** (deviates from the Fable consult's "log even when off"): logging when off would force an IMU read on every pass-through even with the feature disabled. To gather junction-path bench data, flip the junction knob ON and read the skip/fire logs. Recorded in ADR-052's trigger-to-revisit.
- **Parity as a sim-only primitive tag modeled on `ACT_REANCHOR`.** `heading_reset` is NOT a compiler primitive (the compiler stays heading-reset-unaware — `compiler.py` emits nothing for the reset interlude), but it IS a sim-only tag in the tuple stream `PathTrackerSim._walk_hops` yields and `go_to()` returns — exactly like `ACT_REANCHOR` / `ACT_WALL_STOP` today. It must carry the full three-place parity contract or it loud-fails with `ParitySchemaError` "unknown sim ACT tag" (`parity.py:913`). No existing compiler↔sim byte-equality test breaks (no current fixture contains a heading-reset node); this is the identical contract extension `ACT_REANCHOR` already made. Precedent detail: `wall_reanchor`'s parity apply is NOT a no-op (it sets the on-axis coordinate, `parity.py:959-964`) — the precedent covers the record *plumbing*; `heading_reset`'s apply *is* a no-op on pose (the sim's heading is already truth, no drift model).
- **Junction knob will likely end up ON after bring-up, but ships OFF for v1.** Each bind *replaces* accumulated error with one bounded, zero-mean wiggle sample; errors don't compound across resets, so frequent slightly-noisy binds strictly dominate rare clean ones on a long run. But the junction freeze-window envelope is unmeasured, so OFF is the right v1 default; the drift telemetry makes flipping it a data decision.
- **ADR number = next free at land time (052 after the mandatory rebase).** The branch was cut from a stale `main`; `origin/main` already landed ADR-051 (#118). After rebasing onto current `main`, 052 is correct — using 051 would collide (`gotcha-adr-number-collision`).
- **Codex approach consult unavailable** — the `codex:codex-rescue` forwarder runs async/fire-and-forget in this environment (returned task handle `task-mr4k5ka2-orbzkb`, no inline result). The approach was decided on the Fable consult alone (it ENDORSED the design); both the Fable **and Codex** critics ran on the saved spec and their findings are folded in.

## Acceptance Criteria

- [ ] **AC1** — `reanchorHeading(h)` on a successful read with `_navActive` and the strict gate satisfied sets the offset so that `getHeadingDeg() == h` (modulo the live re-read), refreshes `getLastHeadingDeg()` and `getLastHeadingUpdateMs()`, and returns `true`.
- [ ] **AC2** — `reanchorHeading` leaves `getOffsetFromNode()` (x, y, thetaDeg) **byte-for-byte unchanged** — it is heading-only and must not touch `_offset` or the integrator baselines.
- [ ] **AC3** — With `LINE_HEADING_RESET_STRICT_ENABLED == true`, a call whose pre-bind `|belief − h|` **exceeds** `LINE_HEADING_RESET_TOLERANCE_DEG` returns `false` and mutates **nothing** (offset, cache, stamp all unchanged). A difference exactly equal to the tolerance still binds (boundary inclusive, matching `reanchorAtWall`).
- [ ] **AC4** — A failed IMU read returns `false` and mutates nothing; `outPreBindDeltaDeg` is left untouched.
- [ ] **AC5** — `reanchorHeading` before `zeroHeadingFromCurrent` (`!_zeroed`) OR while `!_navActive` returns `false` and mutates nothing.
- [ ] **AC6** — `reanchorAtWall`'s observable behavior is unchanged after the `_bindHeadingFromYaw` extraction: all existing `test_reanchor_at_wall` / `test_reanchor_at_wall_strict` assertions still pass.
- [ ] **AC7** — On a collinear **projection** pass-through with `LINE_HEADING_RESET_ON_PROJECTION == true`, PathTracker binds heading to the incoming edge's `heading_deg`, body-corrected: a REAR/reverse pass-through binds `heading_deg + 180` (the `+180` is computed in PathTracker's `_resolvedBodyHeading`, not in Localization — so this is asserted at the FSM layer, not the unit layer).
- [ ] **AC8** — On a collinear **junction** pass-through, the reset fires **only** when `LINE_HEADING_RESET_ON_JUNCTION == true` (default OFF ⇒ no bind).
- [ ] **AC9** — With `_passThrough` **force-set** true at a RESET node — a state unreachable on any validator-passed field per ADR-048, asserted only to exercise the guard — `_maybeHeadingReset` takes the BLIND/RESET no-op branch and performs no heading reset.
- [ ] **AC10** — A non-pass-through arrival (stop-and-go, BLIND, PROJECTION stop, STOP_AT_LINE seek), a first plan, and a disabled knob each produce **no** heading reset and **no** behavior change (the FSM proceeds exactly as before).
- [ ] **AC11** — The Serial log emits one line on a **fire** (node / kind / heading / pre-bind delta) and one line on a **skip**, where a skip is (a) an IMU-read failure with the knob ON (`delta=n/a`) or (b) a strict-reject with the knob ON (delta + reject reason). Knob-OFF and the BLIND/RESET defensive branch are **silent** — no Serial line. The `heading_reset` **parity trace record is emitted only on a successful bind**, never on a skip.
- [ ] **AC12** — On the **accept path**, `PathTrackerSim` reproduces the firmware's eligibility and fire location and emits a `heading_reset` record with matching `node` / `kind` / `heading_deg`; applying it is a no-op on the virtual chassis pose. The strict-reject path is **firmware-only** (the pure-Python sim has no IMU-drift model and never rejects), verified by host-C++ tests, not the #116 parity harness; parity fixtures are constructed so the firmware does not strict-reject.
- [ ] **AC13** — No change to `tools/path_planner/field_data.json`, `tools/path_planner/field_data.py`, production `WRO2026/routes.h` / `routes.cpp`, or the `ResetNode` / `RESET_NODES` surface; the CI routes-freshness gate stays green with no regeneration.
- [ ] **AC14** — ADR-052 (the next free number after rebasing onto current `main`) exists and is indexed in `DECISIONS.md`; the status-summary table is updated.

## Data Model

No persistent/graph data changes. The only new "shape" is the sim parity record:

```python
# tools/path_planner/sim/parity.py  — TRACE_RECORD_FIELDS entry
"heading_reset": {
    "node":        str,    # node name the reset fired at
    "kind":        str,    # "proj" | "junction"
    "heading_deg": float,  # the world heading bound (body-corrected incoming edge heading)
}
```

Emitted **only on a successful bind** (no `bound`/`result` field — a record's mere presence means it bound). The firmware never serializes state; the reset mutates only the in-RAM `LocalizationClass::_yawOffsetDeg` + heading cache.

## API Contracts

```
LocalizationClass::reanchorHeading(float headingDeg, float* outPreBindDeltaDeg = nullptr) -> bool
  Returns false with NO mutation if !_navActive or !_zeroed.
  Reads the IMU once (via YawSource). If the read fails, returns false, NO mutation, outPreBindDeltaDeg untouched.
  On a successful read: if non-null, sets *outPreBindDeltaDeg to the wrap-correct |normalize(imuYaw - _yawOffsetDeg) - headingDeg|.
  If strict-enabled and that delta > LINE_HEADING_RESET_TOLERANCE_DEG, returns false with NO mutation.
  Else binds via _bindHeadingFromYaw(imuYaw, headingDeg) and returns true. Never touches _offset / integrator baselines.
```

## Contracts & Interfaces

Single source of truth for every seam more than one task touches.

### Signatures

- `LocalizationClass::reanchorHeading(float headingDeg, float* outPreBindDeltaDeg = nullptr) -> bool` — **owner: T2**; consumers: T3 (host test), T4 (PathTracker call site).
- `LocalizationClass::_bindHeadingFromYaw(float imuYaw, float headingDeg)` (private) — **owner: T2**; sets `_yawOffsetDeg = imuYaw - headingDeg`, `_lastGoodHeadingDeg = _normalize(imuYaw - _yawOffsetDeg)`, `_lastGoodHeadingMs = millis()`. Called by both `reanchorHeading` and the refactored `reanchorAtWall` step (1).
- `PathTrackerClass::_maybeHeadingReset(uint8_t cur, bool firstPlan, float incomingBodyHeadingDeg)` (private, void, no `_state` change) — **owner: T4**. `incomingBodyHeadingDeg` is captured from `_resolvedBodyHeading` at the top of `_onPlanNextHop` before any overwrite.
- `PathTrackerClass::_ptEmitHeadingReset(uint8_t node, const char* kind, float headingDeg)` under `#ifdef PT_PARITY_TRACE` (sibling of `_ptEmitWall`; **no `bound` param** — the record is emitted only on a successful bind) — **owner: T4**; its record shape must match the sim emitter (T7) and the `load_cpp_ndjson` loader arm.
- Fire path calls `EventTrace.pushEvent(...)` directly from `_maybeHeadingReset` (owner: T4) — the same call the wall interlude already makes (`PathTracker.cpp:1156/1185`); no new EventTrace accessors, no EventTrace edit.

### Config knobs (all in `namespace Config`, `config/ConfigNav.h`, `_DEFAULT` override-hook idiom)

- `LINE_HEADING_RESET_ON_PROJECTION` : bool, default `true`  — **owner: T1**; consumers: T4, T7.
- `LINE_HEADING_RESET_ON_JUNCTION`   : bool, default `false` — **owner: T1**; consumers: T4, T7.
- `LINE_HEADING_RESET_STRICT_ENABLED`: bool, default `true`  — **owner: T1**; consumers: T2, T3, T7.
- `LINE_HEADING_RESET_TOLERANCE_DEG` : float, default `10.0f` (`static_assert > 0 && < 180`) — **owner: T1**; consumers: T2, T3, T7.

### Parity contract (three-place, model on `ACT_REANCHOR` / `wall_reanchor`)

The `heading_reset` sim-only tag must exist in **all three** places or it loud-fails:
1. Emitted from `PathTrackerSim._walk_hops` at the pass-through fire site (like `_dispatch_wall_reset` yields `ACT_REANCHOR`).
2. A `TRACE_RECORD_FIELDS["heading_reset"]` schema entry **and** a matching `normalize_sim_stream` tag arm in `sim/parity.py` (an unhandled tag → `ParitySchemaError` at `parity.py:913`).
3. The paired C++ emitter (`_ptEmitHeadingReset`) **and** the `load_cpp_ndjson` loader arm.

Any compiler↔sim byte-equality assertion that ever traverses a heading-reset node must interleave/filter the sim-only record (the `_interleave_line_overshoot` pattern in `test_sim_pipeline_parity.py`) so it never enters the compiler-equality comparison.

### Naming (must match across firmware + sim)

- kind strings: `"proj"` (PROJECTION pass-through), `"junction"` (LINE pass-through).
- parity record kind tag: `"heading_reset"`.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/config/ConfigNav.h` | T1 | T2, T4 |
| `WRO2026/Localization.h` / `.cpp` | T2 | T4 |
| `WRO2026/PathTracker.h` / `.cpp` | T4 | — |
| `WRO2026/test/test_reanchor_heading.cpp` | T3 | — |
| `WRO2026/test/test_path_tracker_heading_reset.cpp` | T6 | — |
| `WRO2026/test/CMakeLists.txt` | T3 | T6 (also edits — T6 blocked by T3) |
| `WRO2026/test/fixtures/field_fixture.py` + regen script (only if a triple is missing) | T6 | — |
| `tools/path_planner/sim/runtime.py`, `sim/parity.py` | T7 | T8 |
| `tools/path_planner/tests/…` (sim tests) | T8 | — |
| `docs/decisions/052-*.md`, `DECISIONS.md` | T9 | — |

EventTrace is **not** edited (the fire path calls `EventTrace.pushEvent` from PathTracker, per the wall precedent).

## Error Handling

- **`!_navActive`:** return `false`, mutate nothing (see the Decisions "reject on `!_navActive`" bullet — enforcing the closed-gate invariant, not merely asserting it).
- **`!_zeroed`:** return `false`, mutate nothing (world frame not yet bound).
- **Transient IMU read failure inside `reanchorHeading`:** return `false`, mutate nothing (binding against a stale yaw would skew every later `getHeadingDeg()`), `outPreBindDeltaDeg` untouched. PathTracker logs a skip (`delta=n/a`) with the knob ON and continues the run — no fault.
- **Strict-gate rejection (`|belief − heading| > tolerance`):** return `false`, mutate nothing. PathTracker logs a loud skip with the delta + reason and continues — no fault. The "drifted onto the wrong line / gross wrong state" guard.
- **BLIND/RESET defensive branch (force-set `_passThrough`, unreachable on validator-passed fields):** no bind, no log — a silent no-op.
- **Integrator-gate interaction (validated, must be preserved):** `reanchorHeading` fires while `_navActive == true`, so the operator-offset integrate gate is **closed** and re-baselines correctly on the later closed→open edge from a live read in the new frame. `reanchorHeading` must NOT touch `_offset` / `_gateWasOpen`. See `gotcha-localization-rezero-reopen-edge`.

## Testing Strategy

**Levels:** Unit (host C++ ctest for `reanchorHeading`), Integration (host C++ FSM gating; Python sim parity), Regression (existing wall-reset ctests).

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `reanchorHeading(h)` accept, strict satisfied, nav active | Unit (C++) | `getHeadingDeg()==h`, cache+stamp refreshed, returns `true` (AC1) |
| TC2 | offset untouched by `reanchorHeading` | Unit (C++) | `getOffsetFromNode()` identical before/after (AC2) |
| TC3 | strict reject, `|belief−h| > tol` | Unit (C++) | returns `false`, offset/cache/stamp unchanged, `outDelta` set (AC3) |
| TC4 | strict boundary, `|belief−h| == tol` | Unit (C++) | binds (inclusive), returns `true` (AC3) |
| TC5 | failed IMU read (mock) | Unit (C++) | returns `false`, nothing mutated, `outDelta` untouched (AC4) |
| TC6 | `!_zeroed` guard AND `!_navActive` guard | Unit (C++) | each returns `false`, nothing mutated (AC5) |
| TC7 | `reanchorAtWall` unchanged after extraction | Regression (C++) | all `test_reanchor_at_wall[_strict]` pass (AC6) |
| TC8 | projection pass-through fires (knob ON) | Integration (C++, fixture routes) | binds to incoming `heading_deg` (AC7) |
| TC9 | REAR/reverse projection pass-through | Integration (C++, fixture routes) | binds `heading_deg + 180` — the value delivered to Localization is the body-corrected heading (AC7) |
| TC10 | junction pass-through: OFF target ⇒ no bind; ON target ⇒ bind | Integration (C++, two compile-def targets) | gated by `LINE_HEADING_RESET_ON_JUNCTION` (AC8) |
| TC11 | forced `_passThrough=true` + RESET node ⇒ no reset | Unit (C++, forced state — NOT a validator-passed fixture) | takes the BLIND/RESET no-op branch; note unreachability per ADR-048 (AC9) |
| TC12 | non-pass-through / first-plan / knob-off ⇒ no-op | Integration (C++) | FSM behavior unchanged (AC10) |
| TC13 | Serial line on fire / strict-reject / read-fail / knob-off | Integration (C++, mock Serial) | fire=delta; strict-reject=delta+reason; read-fail=`delta=n/a`; knob-off=**no line** (AC11) |
| TC14 | strict-reject emits NO parity record | Integration (C++) | `_ptEmitHeadingReset` not called on reject; record only on bind (AC11) |
| TC15 | sim accept-path decision + record parity | Integration (Python) | sim fires at same nodes, emits matching `heading_reset` record, no-op on pose (AC12) |
| TC16 | no field/routes drift | Regression (CI) | routes-freshness gate green, no regeneration (AC13) |

**Test data:** Host C++ uses the existing mock IMU/YawSource (inject `imuYaw`, and set `_navActive` via `setNavActive`) and, for the FSM tests, the fixture routes TU (`WRO2026/test/fixtures/routes_fixture.*`). The fixture already carries a collinear projection triple (Inter1–ProjL–Inter2 on y=40) and a collinear LINE row (Inter2–Inter3–Inter4), so **no regeneration is expected**; if a REAR-capable triple or a needed topology is genuinely missing, change it via `field_fixture.py` + the regen script (ADR-050 single-sourcing — never hand-edit `routes_fixture.*`). Resolve nodes **by name** (`FieldMap::findNodeByName`), never index; use explicit `FieldMap` hops if `NEXT_HOP` routes around a triple (`project-host-test-link-toy-routes`). Compile-time knobs are flipped per-target via `target_compile_definitions` (the `test_reanchor_at_wall_strict` `_DEFAULT`-flip idiom). Python sim uses the sim fixtures + the #116 live parity harness.
**Run commands:** `ctest --test-dir build/test --output-on-failure` (host); `uv run --extra dev pytest tools/path_planner/tests/` (Python).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Add the 4 Config knobs | — | low | `WRO2026/config/ConfigNav.h` | Add `LINE_HEADING_RESET_ON_PROJECTION` (default true), `_ON_JUNCTION` (default false), `_STRICT_ENABLED` (default true), `_TOLERANCE_DEG` (default 10.0f), each with the `#ifndef X_DEFAULT / #define / constexpr X = X_DEFAULT` override hook (copy the `WALL_NORMAL_STRICT_ENABLED` idiom, `ConfigNav.h:307-317`). Add `static_assert` on tolerance `>0 && <180` (mirror `WALL_NORMAL_TOLERANCE_DEG`). Doxygen each. Enables T2/T4/T7. |
| T2 | Localization: `_bindHeadingFromYaw` + `reanchorHeading` | T1 | high | `WRO2026/Localization.h`, `WRO2026/Localization.cpp` | Extract the 3-line bind (`Localization.cpp:241-243`) into private `_bindHeadingFromYaw(imuYaw, headingDeg)`; refactor `reanchorAtWall` step (1) to call it (behavior-identical). Add public `reanchorHeading(headingDeg, outPreBindDeltaDeg=nullptr)` per the API contract: guard `!_navActive` then `!_zeroed` (both return false, no mutation), single `_readImuYaw`, fill `outPreBindDeltaDeg` on read success, strict gate using `LINE_HEADING_RESET_STRICT_ENABLED`/`_TOLERANCE_DEG` + `wrapDeltaDeg`, `_bindHeadingFromYaw`, return bool. Do NOT touch `_offset`/baselines. Doxygen: heading-only; the gate is **line-reset-specific** (a future non-line caller must not silently inherit `LINE_HEADING_RESET_*`); call only while the integrate gate is closed (enforced by the `!_navActive` reject). Satisfies AC1–AC6. |
| T3 | Host tests for `reanchorHeading` | T2 | low | `WRO2026/test/test_reanchor_heading.cpp`, `WRO2026/test/CMakeLists.txt` | New ctest target (link `Localization.cpp` like `test_reanchor_at_wall`, `CMakeLists.txt:1102-1115`; add a `_strict`/knob-flip variant via `target_compile_definitions` on the `_DEFAULT` macros to cover strict on/off + tolerance). Cover TC1–TC7. Run full ctest (shared config change can ripple — `gotcha-field-contract-change-breaks-cpp-and-py-fixtures`). |
| T4 | PathTracker: `_maybeHeadingReset` + call site + parity emit + pushEvent | T1, T2 | high | `WRO2026/PathTracker.h`, `WRO2026/PathTracker.cpp` | At the top of `_onPlanNextHop`, capture `const float incomingBody = _resolvedBodyHeading;` before any dispatch. Add `_maybeHeadingReset(cur, firstPlan, incomingBody)` and call it immediately after the `_maybeEnterReset` block (~line 1587), before the destination check. Logic: `if (firstPlan) return; if (!_passThrough) return;` then a `switch(FieldMap::getNodeType(cur))` enumerating **all four** cases — PROJECTION→projection knob & kind "proj"; LINE→junction knob & kind "junction"; BLIND, RESET→return (no-op); **no `default`** so a new enum member trips `-Wswitch`. If the knob is off, return (no read). Else `Localization.reanchorHeading(incomingBody, &delta)`; on `true` → Serial fire log + `EventTrace.pushEvent(...)` (wall-path style, `:1156/1185`) + `#ifdef PT_PARITY_TRACE _ptEmitHeadingReset(cur, kind, incomingBody)` (record ONLY on bind, no `bound` param); on `false` → Serial skip log (`delta` or `n/a`, + reason) only when the knob was ON. No `_state` change. Satisfies AC7–AC12. |
| T6 | Host FSM gating test | T4, T3 | med | `WRO2026/test/test_path_tracker_heading_reset.cpp`, `WRO2026/test/CMakeLists.txt`, (only if a triple is missing) `WRO2026/test/fixtures/field_fixture.py` + regen script | Drive the FSM to the fixture's collinear projection triple (Inter1–ProjL–Inter2) — forward and REAR (TC8/TC9) — and a collinear LINE pass-through (TC10, two compile-def targets for junction OFF/ON). Add the forced-state RESET/BLIND no-op unit case (TC11) and the non-pass-through/first-plan/knob-off no-ops (TC12), and the Serial-line + no-parity-record-on-reject cases (TC13/TC14, mock Serial). Resolve by name; explicit `FieldMap` hops if `NEXT_HOP` routes around a triple. Thin mocks only. Blocked by T3 (both edit `CMakeLists.txt`). |
| T7 | Sim parity mirror | T4 | med | `tools/path_planner/sim/runtime.py`, `tools/path_planner/sim/parity.py` | Mirror the four knobs as module-level constants in `runtime.py` (like `PATH_PASS_THROUGH_ENABLED`, `runtime.py:299`), defaults matching firmware. In `PathTrackerSim._walk_hops`, at the pass-through fire site, yield a sim-only `ACT_HEADING_RESET` (modeled on `ACT_REANCHOR`) **only when the fire decision is true**. Wire the full three-place parity contract (Contracts § Parity): `TRACE_RECORD_FIELDS["heading_reset"] = {node, kind, heading_deg}` + a `normalize_sim_stream` tag arm + the `load_cpp_ndjson` loader arm; a no-op-on-pose apply. Note: the sim has no drift ⇒ never strict-rejects ⇒ mirrors only the ACCEPT/fire decision (the reject path is host-C++-only). |
| T8 | Python sim tests | T7 | low | `tools/path_planner/tests/test_runtime_sim.py` (+ #116 parity harness test) | Assert TC15: sim fires on eligible nodes with matching record fields, skips on first-plan/knob-off, no-op on pose; and that an unhandled tag would loud-fail. Run `uv run --extra dev pytest tools/path_planner/tests/`. |
| T9 | ADR-052 + index | T2, T4, T7 | low | `docs/decisions/052-line-arrival-heading-reset.md`, `DECISIONS.md` | Write ADR-052 (Rule / Why / How to apply / Trigger to revisit / See also). Confirm 052 is the next free number after the rebase (origin/main landed ADR-051 via #118); use the next free number if a parallel PR grabbed 052. Trigger-to-revisit: broaden to every-LINE-arrival reset or a continuous complementary-filter trim (Fable Approach 6), flip the junction knob ON with bench data, add a settledness sample-gate, add always-on feature-off drift telemetry. See also ADR-042/043/045/046/048. Update `DECISIONS.md` index + status table. |

## Notes for Implementer

- **First, rebase the branch onto current `main`** (it landed ADR-051 via #118). This is a hard precondition: it makes ADR-052 the correct number and pulls in the current `WRO2026/test/CMakeLists.txt` (which now has CTPS targets — T3/T6 add to it) and the current `DECISIONS.md` (T9 appends after ADR-051). The 3 landed commits are all CTPS-only and touch none of the core files this feature edits.
- **Pass the incoming heading as a parameter, don't read the member at the fire site.** Capture `_resolvedBodyHeading` into a local at the top of `_onPlanNextHop` before any dispatch overwrites it (`:1757`), and pass it to `_maybeHeadingReset`. Firing before the direction-resolution read (`getHeadingDeg()` at `:1650`) is intentional: the next hop plans against the corrected heading.
- **The BLIND/RESET no-op is defense-in-depth, not a live-bug fix.** Per ADR-048 a RESET pass-through is unreachable on any validator-passed field; the branch guards ADR-048's trigger-to-revisit and validator-bypassing hand-edits. Don't oversell it; test it with forced state (TC11), and don't file a bug about the wall-`RESET`-pass-through / still-live-LineFollower path — it's unreachable, so there's no issue to file.
- **`reanchorHeading` must refresh the cache + stamp** (the shared `_bindHeadingFromYaw` does this) — else the next arrival's `_consumeOffset` snapshots `_lastYawDeg` from a pre-bind cached heading and EventTrace serves stale headings.
- **Do not touch `_offset`.** It was zeroed by the arrival hook one call earlier; a heading-only verb that also cleared it would mask a future call-order bug.
- **Timing:** `reanchorHeading` adds a *second* blocking lower-MCU IMU read in the same tick as the existing `getHeadingDeg()` at `:1650` (~2–3 ms each per the vendor-read profile) while the follower is live mid-pass-through. Acceptable — still well inside one ~28 ms tick — but the "~30 ms / ~1 cm" argument should account for it.
- **The parity record and the Serial log are different channels** — the record fires only on a bind (so the driftless sim matches); the log covers fires and knob-ON skips. Keep them separate.
- **Run BOTH full suites.** A shared `ConfigNav.h` change plus a `Localization.cpp` refactor can ripple into the wall-reset `_strict` sibling and Python fixtures — run the full `ctest` and the full `pytest` (`gotcha-field-contract-change-breaks-cpp-and-py-fixtures`, `gotcha-host-tests-link-production-routes`).
- **Host build gotchas:** MSVC-only ODR/link errors surface under `arduino-cli`, not host ctest (`arduino_cli_headless_available`); toolchain recipe in `host_cpp_toolchain_available`.
- **Production routes are a placeholder.** `N_NODES == 1` today (`project-field-data-is-test-map`), so the feature is **inert on the current production map** — it ships infrastructure-only until the real field geometry has eligible collinear projection pass-throughs. This is expected, not a gap.
- **Rollback:** the feature is inert with both knobs OFF; `reanchorHeading` is reachable only via `_maybeHeadingReset`, and `reanchorAtWall`'s extraction is behavior-preserving (guarded by TC7). Reverting T4 alone disables the feature end-to-end.
