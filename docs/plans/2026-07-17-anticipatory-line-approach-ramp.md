# Anticipatory Line-Approach Ramp Plan

**Goal:** Stop line→line approaches from cruising at full speed until the leading line-sensor board sees the target line and only *then* braking (too late — overshoot / abrupt stop). Instead, predict how far the leading ("vision") board travels before it reaches the target line and ramp speed down (trapezoidal / "T-shape") over that predicted distance, so the robot is already slow when it arrives. Physical line **detection stays the authoritative stop** (it re-anchors `Localization`).

**Approach:** Extract the ramp schedule (predicted distance → per-tick PWM magnitude + phase) into one header-only, host-testable `ApproachRampPolicy.h` that **composes** the already golden-vector-pinned `resolveTShapeRampCm` kernel (`TShapeProfilePolicy.h`). Both reactive-stop consumers call it: the LineFollower junction hop (`PathTracker::LINE_FOLLOW_INTERSECTION`) and the `Drive.stopAtLine` blind seek (SEEK/ARMED phases). The Python CTPS mirror (`profile.py`) reproduces the same arithmetic pinned by cross-language golden vectors, and the CTPS player renders the ramp. The prediction is the **pivot-trimmed edge length** (`PathTracker::_pivotTrimDistance(edge)`, *not* `commandedDistanceCm` — see C1) **minus the direction-aware sensor-to-axle offset**; the ramp bottoms out a tunable margin *before* the prediction and holds the kinetic floor until detection; if detection never comes within a bounded overrun, a hard fault fires. Default-on with a global Config toggle that reproduces today's flat behavior byte-for-byte.

## Scope

- **In scope:**
  - `ApproachRampPolicy.h` — new header-only pure policy (resolve-once schedule + speed-at pair), composing `resolveTShapeRampCm`.
  - Anticipatory ramp on the **LineFollower junction hop** (`PathTracker::LINE_FOLLOW_INTERSECTION` → `State::LINE_FOLLOW`), driven by the predicted sensor-crossing distance.
  - Anticipatory ramp on the **`Drive.stopAtLine` seek** (SEEK/ARMED phases).
  - New bounded-overrun fault on the LineFollower hop (it has none today); the seek keeps its existing `LINE_NOT_FOUND` max-margin fault.
  - Monotonic-PWM guarantee across the detect→overshoot handoff on **both** paths (fixes a latent "jolt-back-to-cruise" defect).
  - Hoisting the collinear pass-through decision to dispatch time so the ramp is suppressed on pass-through nodes.
  - Updating the four host-test `LineFollower.h` mock headers to mirror the new builder surface (else every mock-linked target fails to compile / silently false-greens).
  - Three new `Config` knobs (enable toggle, pre-prediction margin, overrun margin) + `ConfigChecks.h` asserts.
  - Full offline parity: `profile.py` mirror + cross-language golden vectors + `PathTrackerSim` parity + CTPS ramp rendering.
  - ADR-067 (verify next free number) + `DECISIONS.md` index update.
- **Out of scope:**
  - Refactoring the existing tuned `PROJECTION_FOLLOW` / `LINE_OVERSHOOT` / seek-`OVERSHOOT` ramp-down code onto the new policy — they stay byte-identical this PR (deferred cleanup, see Notes).
  - Any change to the ramp *slope* or *bounds* — the feature reuses `TSHAPE_ACCEL_PWM_PER_CM` + `TSHAPE_RAMP_MIN/MAX_CM` unchanged.
  - Changing the arrival floor value — it reuses the existing kinetic floor `PWM_MOVING_MIN`.
  - Reduced-cruise degrade for short hops (below the engage threshold the hop runs flat — a documented known limit, see the short-hop Decision; a triangle-style reduced-peak fallback is a deferred follow-up).
  - Curved-path chord-vs-arc-length correction of the encoder prediction (handled by the safety margin + bounded overrun, not by geometry).
  - Blind dead-reckon hops, wall-stop, return-to-node — those are distance-gated, not reactive-stop, and already decelerate.

## Decisions

- **Both reactive-stop paths in scope** — the identical defect exists on the LineFollower junction hop and the `Drive.stopAtLine` seek. (User.)
- **Detection stays authoritative** — the ramp is anticipatory only; `Intersection.isAtIntersection()` (LineFollower) and the `T_OR_CROSS` crossing (seek) still trigger the stop + `Localization` re-anchor, so encoder-prediction error is self-correcting. (User.)
- **Default-on + global toggle** — `Config::LINE_APPROACH_RAMP_ENABLED = true`; when `false` the code path is byte-identical to today's flat behavior (HOP_SETTLE_MS==0 precedent, ADR-061). (User.)
- **Ramp bottoms out at the kinetic floor `PWM_MOVING_MIN` (~30)** — the same floor the existing ramp-down targets; no new arrival-speed constant. (User.)
- **Bottom out a tunable margin BEFORE the prediction** — `Config::LINE_APPROACH_MARGIN_CM`, so a line arriving early still meets an already-slow robot. (User.)
- **Late line → bounded search then hard fault** — advance at floor past the prediction for `Config::LINE_APPROACH_MAX_MARGIN_CM`; if still undetected, raise `LINE_NOT_FOUND` / `DRIVE_FAULT`. Mirrors the seek's existing max-margin fault (ADR-039: a missed line is a hard fault, not a silent crawl). (User.)
- **Pass-through collinear nodes are excluded** — the ramp fires only on a hop that *terminates* at the target node; full cruise is preserved across a node driven straight through (ADR-023/059). (User.)
- **Short hops below the engage threshold run flat (known limit)** — with `TSHAPE_ACCEL_PWM_PER_CM=4`, cruise ~80→floor 30 needs a ~12.5 cm ramp; plus `LINE_APPROACH_MARGIN_CM` (4) + front offset (12.02) the *forward* engage threshold is ~28-29 cm axle-frame. Hops shorter than that degrade to today's flat behavior (`ApproachSchedule.active=false`). The user confirmed flat-fallback for short hops; this decision only makes the threshold explicit and documented (ADR trigger-to-revisit: if short junction spacing on the real mat leaves too many hops unimproved, add a reduced-cruise/triangle fallback). (User-confirmed assumption + critic.)
- **Shared header-only policy composing the pinned T-shape kernel** — one source of truth across firmware + sim + CTPS, matching the `TShapeProfilePolicy.h` / golden-vector convention (ADR-058). (User pick + Fable endorse.)
- **Resolve-once schedule POD + speed-at pair** (not a monolithic per-tick function) — the schedule POD is the artifact CTPS renders and the parity trace records. (Fable amendment.)
- **Approach ramp is sub-FSM; the parity trace is NOT extended** — the anticipatory ramp shapes the LineFollower's per-tick speed *below* the FSM parity layer, so the ADR-050 dispatch trace (states + primitives) is byte-identical and parity stays green with **no new record `kind`** (this also keeps the T8→T9 ordering clean — T8 can't emit a kind T9 defines). The per-tick *dispatched* magnitude is verified by a host integration test (T11), and `PathTrackerSim` (T9) mirrors the deceleration in its motion/pose model via `profile.py`'s `approach_speed_at` for CTPS rendering only — matching ADR-058 (CTPS renders the profile from `profile.py`, not from trace records). (Critic W3, resolved.)
- **Prediction source is `_pivotTrimDistance(edge)`, not `commandedDistanceCm`** — `commandedDistanceCm` is `0.0f` by design for `LINE_FOLLOW_INTERSECTION` (`EdgeDispatchPolicy.h:169`; it is sensor-terminated), so it cannot seed the prediction. Use the pivot-trimmed edge length (the same value `PROJECTION_FOLLOW` drives). (Critic C1.)
- **Do NOT re-parameterize the existing gate ramp-down** — the approach ramp is a *parallel* branch; `PROJECTION_FOLLOW` and seek `OVERSHOOT` stay byte-identical. (Fable amendment F4.)
- **Config knobs live in `ConfigDrive.h`** — co-located with `TSHAPE_*`, `LINE_SENSOR_AXLE_OFFSET_CM`, and the `stopAtLine` seek block this feature extends.
- **Review provenance:** Fable **endorsed** the shared-policy strategy (approach consult) with four amendments (all folded in). Codex consult + Codex critic were dispatched but their background results were not retrievable in-session; the Fable critic returned **2 critical, 6 warning, 4 nit**, all accepted. Both criticals were verified directly against the source (`EdgeDispatchPolicy.h:169`; the four `LineFollower.h` mocks). Re-run a fresh Codex pass if cross-vendor confirmation is wanted before implementation.

## Acceptance Criteria

- [ ] **AC1 — Anticipatory decel (LineFollower).** On a terminating LINE→line hop with a known edge distance, commanded PWM begins ramping down before the leading sensor detects the marker; at the predicted sensor-crossing distance the commanded magnitude is at the kinetic floor `PWM_MOVING_MIN` (± the ramp granularity), not full cruise. (Covered by TC1, TC10.)
- [ ] **AC2 — Anticipatory decel (seek).** `Drive.stopAtLine` ramps its SEEK/ARMED magnitude from `_stopCruise` down to the floor before the expected crossing, instead of firing constant cruise until detection. (TC2, TC11.)
- [ ] **AC3 — Monotonic detect handoff (both paths).** Across the tick where the line is detected and the FSM transitions to the overshoot leg, the commanded PWM magnitude is **non-increasing** — the robot never speeds back up on detection. (TC3, TC12 — the F2 fix.)
- [ ] **AC4 — Detection stays authoritative + re-anchor.** The physical crossing still terminates the hop and re-anchors `Localization` (LineFollower via `onIntersection`; seek via its existing re-anchor). Encoder distance is never the *nominal-arrival* stop trigger; encoder *bounds* trigger only the overrun/max-margin *fault*. (TC4.)
- [ ] **AC5 — Pass-through not slowed.** A collinear pass-through node keeps full cruise across the node — the approach ramp is suppressed. (TC5.)
- [ ] **AC6 — Bounded overrun fault (LineFollower).** If no line is detected by `predicted + LINE_APPROACH_MAX_MARGIN_CM`, PathTracker enters its fault path (`LINE_NOT_FOUND`/`DRIVE_FAULT`), stopping the chassis rather than crawling forever. (TC6.)
- [ ] **AC7 — Short-hop flat fallback.** When the predicted approach distance is below the engage threshold (see the short-hop Decision), the hop silently runs flat (today's behavior); no caller special-casing. (TC7.)
- [ ] **AC8 — Toggle-off is byte-identical.** Forcing `LINE_APPROACH_RAMP_ENABLED = false` leaves both paths running today's flat + reactive behavior — asserted by the existing suites passing unchanged with the toggle off (no separate golden capture). (TC8.)
- [ ] **AC9 — Direction-aware prediction.** Forward hops use `LINE_SENSOR_AXLE_OFFSET_CM` (12.02); reverse hops use `LINE_SENSOR_AXLE_OFFSET_REAR_CM` (7.68), via the `axleOffsetCm` PathTracker already resolves. (TC9 — integration through dispatch.)
- [ ] **AC10 — Cross-language parity.** `profile.py`'s `approach_speed_at` / `resolve_approach_schedule` match the C++ `ApproachRampPolicy` golden vectors bit-for-bit (float32 contract). (TC13.)
- [ ] **AC11 — Sim FSM parity green.** The ADR-050 sim↔firmware parity job stays green — the FSM dispatch trace is unchanged (approach ramp is sub-FSM) and the sim's pose model is distance-driven, so parity holds with **no `PathTrackerSim` change**. CTPS renders the deceleration from `profile.py` in `track_builder.py`. (TC14.)
- [ ] **AC12 — CTPS renders the ramp.** The CTPS player shows the anticipatory deceleration segment on a line→line hop. (Manual eyeball + TC15.)
- [ ] **AC13 — Existing tuned paths unchanged.** `PROJECTION_FOLLOW`, `LINE_OVERSHOOT`, seek `OVERSHOOT`, wall-stop, and blind-chain behavior are byte-identical (their golden/host tests still pass with no golden regeneration). (Full ctest + pytest.)

## Data Model

Not applicable (firmware behavior; no persisted schema). The one new in-memory structure is the policy POD, pinned below.

## Contracts & Interfaces

Single source of truth for every cross-task seam. Owners produce; consumers conform byte-for-byte.

### `ApproachRampPolicy.h` (owner: T2)

Header-only, `#include <stdint.h>` only, zero Arduino/Config/MatrixMiniR4 surface (knobs passed in), pre-`deadbandSnap`, magnitude-domain (sign applied by the caller) — exactly the `TShapeProfilePolicy.h` conventions. Behavior-identity contract (float32 arithmetic, `int16_t` truncation) pinned by golden vectors.

```cpp
enum class ApproachPhase : uint8_t { CRUISE, RAMP, FLOOR_HOLD, OVERRUN };

struct ApproachSchedule {
    float   rampStartCm;      ///< traveled where ramp begins (floorStartCm - rampLenCm); <=0 => degrade-to-flat
    float   floorStartCm;     ///< traveled where floor hold begins (predictedCm - marginCm)
    float   overrunLimitCm;   ///< traveled where OVERRUN begins (predictedCm + maxMarginCm)
    int16_t cruiseMag;        ///< cruise magnitude (>=0)
    int16_t floorMag;         ///< arrival/hold magnitude (>=0, == PWM_MOVING_MIN)
    bool    active;           ///< false => policy degraded to flat (short hop / disabled); caller runs today's path
};

struct ApproachSpeed {
    int16_t       pwmMag;     ///< commanded magnitude for this traveled distance (pre-snap, unsigned)
    ApproachPhase phase;      ///< CRUISE | RAMP | FLOOR_HOLD | OVERRUN
};

// Composes resolveTShapeRampCm(cruiseMag, floorMag, accel, rampMin, rampMax, noRampEps) for rampLenCm.
// NOTE (impl): the header takes no Config surface, so TSHAPE_MIN_MOVE_CM is passed in as the trailing
// `minMoveCm` parameter (10 params total). Every consumer (Python mirror, golden dump, callers) passes it.
inline ApproachSchedule resolveApproachSchedule(
    float predictedCm, float marginCm, float maxMarginCm,
    int16_t cruiseMag, int16_t floorMag,
    float accel, float rampMin, float rampMax, int16_t noRampEps, float minMoveCm);

inline ApproachSpeed approachSpeedAt(const ApproachSchedule& s, float traveledCm);
```

- Owner: T2. Consumers: T4 (Python mirror — same fields/order), T6 (LineFollower), T7 (Drive seek), T3 (golden dump).
- `active == false` (schedule degenerate: `rampStartCm <= 0` or `predictedCm < TSHAPE_MIN_MOVE_CM`) means the caller runs its existing flat path — the policy never fabricates a partial ramp.
- **OVERRUN pwmMag is pinned to `floorMag`** — past `overrunLimitCm` the policy keeps returning the floor magnitude (phase `OVERRUN`); *stopping* is the caller's fault action, not the policy's.

### New `Config` constants (owner: T1) — `WRO2026/config/ConfigDrive.h`

```cpp
constexpr bool  LINE_APPROACH_RAMP_ENABLED = true;   // false => byte-identical to today
constexpr float LINE_APPROACH_MARGIN_CM    = 4.0f;   // bottom out this far BEFORE the prediction
constexpr float LINE_APPROACH_MAX_MARGIN_CM = 15.0f; // LineFollower overrun bound before fault (parallels STOP_AT_LINE_MAX_MARGIN_CM)
```

- Consumers: T6, T7, T8 (firmware), T4 (`profile.py` via `config_parse`), `ConfigChecks.h` (T1 asserts: `MARGIN_CM >= 0`, `MAX_MARGIN_CM > 0`).
- Arrival floor = existing `Config::PWM_MOVING_MIN` (no new constant). Slope/bounds = existing `TSHAPE_*`.

### LineFollower new surface (owner: T6) — `WRO2026/src/drive/LineFollower.h/.cpp`

- Builder knob `LineFollowerEnableBuilder& setApproachTarget(float predictedCm)` — arms the approach schedule for this enable session (implies enabled; magnitude domain; direction from the session's reverse flag). Suppressed when `LINE_APPROACH_RAMP_ENABLED == false` or the schedule degrades → flat.
- Query `ApproachPhase approachPhase() const` — read by PathTracker to detect the overrun→fault condition. Read-only. **This is the single pinned query name** (no `isApproachOverrun` alias).
- **Arm lifecycle:** a `setEnabled(true)` apply *without* `setApproachTarget` **disarms** any prior approach schedule — so the `LINE_OVERSHOOT` re-arm (which does not chain `setApproachTarget`) clears it; no stale `FLOOR_HOLD`/`OVERRUN` leaks into the overshoot leg or the next hop.
- Internal: the approach branch runs **alongside** `_useTShapeRampDown` (never both meaningfully active on one hop; the LINE hop carries no distance gate today). Precedence: approach wins when armed.
- **Detect-handoff seed contract (AC3):** when the hop used an approach ramp, the `LINE_OVERSHOOT` re-arm must seed its ramp-down from the *last commanded magnitude* (floor), not `presetToPwm(_preset)` — so PWM is non-increasing across the detect tick.

### Drive seek surface (owner: T7) — `WRO2026/src/drive/Drive.h/.cpp`

- Internal only: `_updateStopAtLine` SEEK/ARMED feed `approachSpeedAt(...).pwmMag` (sign-applied) to `driveContinuous` instead of constant `_stopCruise`.
- **Seek prediction frame (pinned):** `predictedCm = _stopExpectedCm − axleOffsetCm` (direction-aware; `_stopExpectedCm` is axle-frame — `EdgeDispatchPolicy.h:~183` passes `commandedDistanceCm = rawDistanceCm`, offset NOT subtracted — so the sensor crossing is `offset` cm earlier). An implementer using `_stopExpectedCm` directly would bottom out ~12 cm late; do NOT.
- The seek's own `_stopMaxCm` → `LINE_NOT_FOUND` fault and arm-window/detect-debounce are unchanged (it keeps its existing overrun bound; no new fault added).
- **Detect-handoff seed (AC3):** the `OVERSHOOT` phase seeds its ramp from the **last commanded magnitude** (which equals `_stopCruise` when no approach ramp ran → byte-identical when the feature is disabled), not from `_stopCruise` unconditionally.

### PathTracker (owner: T8) — `WRO2026/src/mission/PathTracker.cpp/.h`

- **Dispatch-time pass-through latch:** hoist the LINE-hop collinear peek to dispatch (mirror the `_projPassThrough` arm-time latch pattern at PathTracker.cpp:~1130 and the shared `_passThroughHeadingOk` predicate at ~2261), latch it, and consume the same latch at detection (LINE_FOLLOW, ~948-993) — evaluate once, consume twice.
- `LINE_FOLLOW_INTERSECTION` dispatch (~2665-2690): when NOT pass-through and `LINE_APPROACH_RAMP_ENABLED`, chain `.setApproachTarget(_pivotTrimDistance(edge) − axleOffsetCm)` onto the existing `setEnabled(true)…` arm. **Prediction source is `_pivotTrimDistance(edge)` (the pivot-trimmed edge length, as `PROJECTION_FOLLOW` uses at ~2711) — NOT `commandedDistanceCm`, which is `0.0f` for this primitive (C1).** `axleOffsetCm` is the direction-aware offset already resolved at ~2596-2597.
- `State::LINE_FOLLOW`: continue to stop on `isAtIntersection()` (authoritative); additionally, poll `LineFollower.approachPhase() == OVERRUN` → enter the existing fault path.
- **No parity-trace change:** the LINE_FOLLOW FSM dispatch record stays byte-identical — the approach ramp is a LineFollower internal detail below the parity layer. Do NOT add a `PathTrackerParityTrace.h` field/kind.

### Host-test mock headers (owner: T13) — the four `LineFollower.h` mocks

`WRO2026/test/{mission_mocks,path_mocks,ctps_mocks,task_mocks}/LineFollower.h` mirror the enable-builder surface. They MUST gain a **recording** `setApproachTarget(float)` (stage the value so `ctps_dump` and integration tests can read it) and a **scriptable** `approachPhase()` (test can drive it to `OVERRUN` for TC6). A lazy no-op stub compiles but silently false-greens TC5/TC6/TC8 and drops the knob from `ctps_dump` (no-op-mock-setter gotcha) — do not stub as no-op.

### Parity + CTPS seams

- `profile.py` (owner T4): `resolve_approach_schedule` + `approach_speed_at` — same field names/order as the POD, float32 mirror.
- Golden dump TU (owner T3): `WRO2026/test/approach_ramp_dump/` (or extend `tshape_profile_dump/`) + freshness `.cmake` — emits the golden vectors.
- Cross-language test (owner T5): `tools/path_planner/tests/test_ctps_profile.py` — pins T4 to the golden.
- Parity: **no** new `PathTrackerParityTrace.h` record kind. The FSM dispatch trace is unchanged (approach ramp is sub-FSM), so ADR-050 parity stays green; `PathTrackerSim` (T9) mirrors the approach deceleration in its motion/pose model only. The dispatched per-tick magnitude is asserted by a host test (T11).
- CTPS renderer (owner T10): `track_builder.py` / player assets render the approach ramp segment (display-only, ADR-058 layer).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/src/drive/ApproachRampPolicy.h` | T2 | T3, T4, T6, T7 |
| `WRO2026/config/ConfigDrive.h`, `ConfigChecks.h` | T1 | T6, T7, T8, T4 |
| `WRO2026/src/drive/LineFollower.h/.cpp` | T6 | T8, T11, T13 |
| `WRO2026/src/drive/Drive.h/.cpp` | T7 | T11 |
| `WRO2026/src/mission/PathTracker.cpp/.h` | T8 | T9, T11 |
| `WRO2026/test/{mission_mocks,path_mocks,ctps_mocks,task_mocks}/LineFollower.h` | T13 | T8, T10, T11 |
| `WRO2026/test/**` (dump TU, CMake, integration tests) | T3, T11 | — |
| `tools/path_planner/ctps/profile.py` | T4 | T5, T10 |
| `tools/path_planner/tests/test_ctps_profile.py` + golden | T5 | — |
| `tools/path_planner/sim/**` | T9 | T10 |
| `tools/path_planner/ctps/track_builder.py` + player assets | T10 | — |
| `docs/decisions/067-*.md`, `DECISIONS.md` | T12 | — |

### Naming (exact strings that must match across tasks)

- Policy: `ApproachRampPolicy.h`, `ApproachSchedule`, `ApproachSpeed`, `ApproachPhase{CRUISE,RAMP,FLOOR_HOLD,OVERRUN}`, `resolveApproachSchedule`, `approachSpeedAt`.
- Python mirror: `resolve_approach_schedule`, `approach_speed_at` (snake_case), same field names.
- Config: `LINE_APPROACH_RAMP_ENABLED`, `LINE_APPROACH_MARGIN_CM`, `LINE_APPROACH_MAX_MARGIN_CM`.
- LineFollower verb: `setApproachTarget`; query: `approachPhase` (the only query name — no `isApproachOverrun`).

## Error Handling

- **Line never detected (LineFollower hop):** advance at floor to `predicted + LINE_APPROACH_MAX_MARGIN_CM`, then `approachPhase() == OVERRUN`; PathTracker raises `LINE_NOT_FOUND` (stop + clear queue). This is a **new** safety bound the LINE hop lacked (previously an unbounded follow).
- **Line never detected (seek):** unchanged — the existing `stopAtLineSeekMaxedOut(traveled, _stopMaxCm)` → `Fault::LINE_NOT_FOUND` still fires; the ramp only changes the magnitude fed to `driveContinuous`.
- **Line detected early (mid-ramp):** proceed to the existing overshoot leg from the current (already-reduced) magnitude — strictly better than today; the AC3 seed makes it monotonic.
- **Short / degenerate hop:** `ApproachSchedule.active == false` → caller runs its existing flat path.
- **`accel <= 0`:** precondition (like `TShapeProfilePolicy.h`) — the caller guards; the policy performs no divide guard of its own. Config `TSHAPE_ACCEL_PWM_PER_CM` is a positive constant, so this is a defensive note only.
- **Junction PID freeze during the ramp (ADR-020):** the freeze commands symmetric `deadbandSnap(cruise)` where `cruise` is the ramped magnitude computed earlier in `update()`, so a freeze mid-ramp holds the *ramped-down* speed, not full cruise. Verify ordering is preserved (approach magnitude computed before the freeze branch).

## Testing Strategy

**Levels:** Unit (policy, host ctest), Integration (host ctest linking real `Drive.cpp`/`PathTracker.cpp`/`LineFollower.cpp`), Cross-language golden (pytest), Sim parity (pytest, CI-enforced), Hardware bench (B-series).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `approachSpeedAt` over a swept traveled range for a nominal hop | Unit | CRUISE → RAMP (monotone decreasing) → FLOOR_HOLD at `PWM_MOVING_MIN` → OVERRUN (returns `floorMag`) past the bound |
| TC2 | Seek SEEK/ARMED magnitude vs traveled (host, real `Drive.cpp`) | Integration | Ramps `_stopCruise`→floor so the floor is reached by the **sensor-frame** crossing (`_stopExpectedCm − axleOffsetCm`), constant floor through ARMED |
| TC3 | Detect tick: last approach magnitude vs first overshoot magnitude (LineFollower) | Integration | overshoot magnitude ≤ approach floor magnitude (non-increasing) — AC3 |
| TC4 | Line detected → hop terminates + `Localization.onIntersection` fires | Integration | Stop is detection-triggered; node advanced; encoder never the *nominal* trigger |
| TC5 | Collinear pass-through hop with approach ramp enabled | Integration | Ramp suppressed; full cruise held across node; `_passThrough` latch consistent dispatch↔detect |
| TC6 | Line removed; run to `predicted + MAX_MARGIN` (LineFollower); mock `approachPhase` drives `OVERRUN` | Integration | `approachPhase()==OVERRUN` → PathTracker `LINE_NOT_FOUND`; chassis stopped |
| TC7 | Hop shorter than the engage threshold | Unit+Integration | `ApproachSchedule.active==false`; flat behavior; no partial ramp |
| TC8 | `LINE_APPROACH_RAMP_ENABLED=false` on both paths | Integration | Byte-identical to today (no `setApproachTarget` arm; flat cruise until detection) |
| TC9 | Forward vs reverse LINE hop through PathTracker dispatch | Integration | Prediction uses 12.02 (front) vs 7.68 (rear) via `axleOffsetCm` — AC9 |
| TC10 | Full LINE→line hop trace (host PathTracker) | Integration | Speed profile has a decel segment before detection; matches AC1 |
| TC11 | Seek `OVERSHOOT` seed after an approach ramp | Integration | Overshoot ramp seeds from last commanded magnitude, not `_stopCruise` (AC3, seek side) |
| TC12 | Seek monotonic detect handoff | Integration | Non-increasing PWM across the confirm tick |
| TC13 | Cross-language golden: C++ dump vs `profile.py` | Golden (pytest) | Bit-for-bit float32 match on `resolve_approach_schedule`/`approach_speed_at` |
| TC14 | ADR-050 sim↔firmware parity job over a route exercising a LINE→line hop | Parity (pytest) | Trace matches; sim arms + records the same schedule; a host test asserts dispatched magnitude |
| TC15 | CTPS render of a line→line hop | Golden + eyeball | Approach ramp segment present in the rendered profile |
| TC16 | Existing ramp-down / overshoot / wall-stop / blind-chain suites | Regression | All pass with NO golden regeneration (AC13) |
| TC17 | Policy `predictedCm` plumbs through unchanged (offset already applied by caller) | Unit | `resolveApproachSchedule` treats `predictedCm` as the sensor-crossing distance; no offset math inside the policy |

**Test data:** host ctest mock/real HAL harness (ADR-030/049); toy field fixtures that include a non-axis-aligned turn so gates use `NAV_TURN_TOLERANCE_DEG` (avoid the axis-aligned-hides-subtolerance gotcha); golden vectors emitted by the dump TU; parity route fixture per ADR-050.
**Run commands:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`; `uv run --extra dev pytest tools/path_planner/tests/`; regenerate routes only if field fixtures change.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Add Config knobs + asserts | — | low | `WRO2026/config/ConfigDrive.h`, `ConfigChecks.h` | Add `LINE_APPROACH_RAMP_ENABLED`, `LINE_APPROACH_MARGIN_CM`, `LINE_APPROACH_MAX_MARGIN_CM` with Doxygen (co-located with `TSHAPE_*` / stopAtLine block); add `ConfigChecks.h` static_asserts (`MARGIN_CM>=0`, `MAX_MARGIN_CM>0`). If `profile.py` reads them, keep `#define`/comment layout `config_parse`-friendly (trailing-comment gotcha). Satisfies part of AC8/AC6. |
| T2 | `ApproachRampPolicy.h` header-only policy | — | med | `WRO2026/src/drive/ApproachRampPolicy.h` | Implement `resolveApproachSchedule` (composing `resolveTShapeRampCm`) + `approachSpeedAt` + PODs per Contracts. `<stdint.h>` only, pre-snap, magnitude domain, OVERRUN→floorMag, float32/`int16_t`-truncation behavior-identity contract documented. Satisfies AC1 math, AC7 degrade, TC17. |
| T3 | Policy unit tests + golden dump TU | T2 | low | `WRO2026/test/approach_ramp_dump/*`, `WRO2026/test/test_approach_ramp_policy.cpp`, `WRO2026/test/CMakeLists.txt` | ctest unit for the policy (phases, monotonicity, degrade, OVERRUN=floor, `predictedCm` pass-through) + a golden-vector dump TU + freshness `.cmake` (mirror `tshape_profile_dump`). Orchestrator owns CMakeLists appends. Satisfies TC1, TC7, TC17. |
| T4 | Python mirror in `profile.py` | T2 | med | `tools/path_planner/ctps/profile.py` | Add `resolve_approach_schedule` + `approach_speed_at` (float32 mirror, same field names/order). Wire Config values via `config_parse`. Satisfies AC10 (mirror half). |
| T5 | Cross-language golden test | T3, T4 | low | `tools/path_planner/tests/test_ctps_profile.py`, `tools/path_planner/tests/golden/*` | pytest pinning `profile.py` to the C++ golden vectors bit-for-bit. Satisfies AC10, TC13. |
| T6 | LineFollower approach-ramp consumer | T1, T2 | high | `WRO2026/src/drive/LineFollower.h/.cpp` | Add `setApproachTarget(cm)` builder knob + `approachPhase()` query; disarm the schedule on a plain `setEnabled(true)` apply; run the approach branch **alongside** `_useTShapeRampDown` (precedence: approach when armed); implement the AC3 detect-handoff seed (LINE_OVERSHOOT seeds from last commanded magnitude). Do NOT alter the existing `_useTShapeRampDown` math (F4). Satisfies AC1, AC3, AC6-signal, AC9. |
| T7 | Drive.stopAtLine seek consumer | T1, T2 | high | `WRO2026/src/drive/Drive.h/.cpp` | Feed `approachSpeedAt(...)` to `driveContinuous` in SEEK/ARMED using `predictedCm = _stopExpectedCm − axleOffsetCm`; seed the `OVERSHOOT` ramp from the last commanded magnitude (== `_stopCruise` when disabled, AC3). Leave arm-window / detect-debounce / `_stopMaxCm` fault / wall-stop untouched (F4). Satisfies AC2, AC3(seek), AC13(seek). |
| T13 | Update the four LineFollower host-test mocks | T6 | med | `WRO2026/test/{mission_mocks,path_mocks,ctps_mocks,task_mocks}/LineFollower.h` | Mirror the new builder surface: recording `setApproachTarget(float)` (stage the value) + scriptable `approachPhase()`. Must NOT be a no-op stub (false-green + drops the `ctps_dump` knob). Unblocks T8 compilation + T10/T11. |
| T8 | PathTracker dispatch + pass-through hoist + overrun fault | T6, T13 | high | `WRO2026/src/mission/PathTracker.cpp/.h` | Hoist the LINE-hop collinear pass-through peek to dispatch, latch, consume at both dispatch and detect (F3, evaluate-once); arm `setApproachTarget(_pivotTrimDistance(edge) − axleOffsetCm)` on terminating LINE hops when enabled (NOT `commandedDistanceCm`, C1); poll `approachPhase() == OVERRUN` → existing fault path; keep the LINE_FOLLOW FSM dispatch trace byte-identical (no parity-trace change); keep detection authoritative. Satisfies AC4, AC5, AC6. |
| T9 | Sim parity (`PathTrackerSim`) | T8, T4 | high | `tools/path_planner/sim/*` | Confirm the ADR-050 parity job stays GREEN (the FSM dispatch trace is unchanged and the sim pose is distance-driven — the approach ramp is sub-FSM). Outcome: **no sim change is needed** — CTPS (T10) renders the deceleration in `track_builder.py` from geometry + `profile.py`'s `approach_speed_at`, reading the sim's LINE-hop poses. Confirm the seam; do NOT add a `PathTrackerParityTrace.h` record kind. Satisfies AC11, TC14. |
| T10 | CTPS ramp rendering | T4, T9, T13 | med | `tools/path_planner/ctps/track_builder.py`, player assets | Render the anticipatory approach ramp segment (display-only, ADR-058 layer); reuse the profile mirror. Satisfies AC12, TC15. |
| T11 | Host integration + regression tests | T6, T7, T8, T13 | med | `WRO2026/test/test_path_tracker_*.cpp`, `test_line_follower_overshoot.cpp`, `test_path_tracker_stop_at_line.cpp`, CMake | Extend integration tests for AC2-AC9/AC13: monotonic handoff (per-tick PWM already asserted in `test_line_follower_overshoot.cpp:117,292`), pass-through exclusion, overrun→fault, toggle-off, seek ramp (sensor frame), forward-vs-reverse offset (TC9), dispatched-magnitude assertion (TC14 host half), no regression on tuned paths. Orchestrator owns CMakeLists appends. Satisfies TC2-TC12, TC16. |
| T12 | ADR-067 + DECISIONS.md | T8 | low | `docs/decisions/067-anticipatory-line-approach-ramp.md`, `DECISIONS.md` | Write the ADR (rule/why/how/trigger/status); **verify the next free ADR number** against the uncommitted `2026-07-17-external-gyro-yaw-source` plan / other open worktrees (collision gotcha); include the short-hop engage-threshold as the trigger-to-revisit; update the DECISIONS.md index + status table. |

## Notes for Implementer

- **C1 — do NOT use `commandedDistanceCm` as the prediction source.** For `LINE_FOLLOW_INTERSECTION`, `EdgeDispatchPolicy.h:169` sets `commandedDistanceCm = 0.0f` by design (sensor-terminated). Seed the prediction from `PathTracker::_pivotTrimDistance(edge)` (the pivot-trimmed edge length, as `PROJECTION_FOLLOW` uses) minus the direction-aware `axleOffsetCm`. Getting this wrong makes the schedule degenerate → the ramp never fires (silent no-op of the whole feature).
- **C2 — the four `LineFollower.h` mocks must mirror the surface (T13).** `mission_mocks`, `path_mocks`, `ctps_mocks`, `task_mocks` each mock the enable builder; adding `setApproachTarget`/`approachPhase` to PathTracker breaks their compile until mirrored, and a no-op stub silently false-greens TC5/TC6/TC8 and drops the knob from `ctps_dump`. Record the value; make `approachPhase` scriptable.
- **F1 — compose, don't re-derive.** `ApproachRampPolicy` must call `resolveTShapeRampCm` from `TShapeProfilePolicy.h` for the ramp length — no fourth copy of ramp arithmetic. The only new math is three phase boundaries + one lerp.
- **F2 — the detect handoff is load-bearing (AC3).** Today `LineFollower._applyEnableBuilder` seeds `_tShapeRampDownStart` from `presetToPwm(_preset)` (LineFollower.cpp:726), and the seek `OVERSHOOT` starts `rampMag` at `_stopCruise` (Drive.cpp:1320-1329). With an approach ramp the robot arrives at floor; if the overshoot leg re-seeds from full preset the robot **jolts back to cruise** on detection. Seed both overshoot legs from the last commanded magnitude when an approach ramp was active.
- **F3 — pass-through peek at dispatch.** The LINE-hop `_passThrough` is decided at detection today (PathTracker.cpp:948-995); `_passThroughHeadingOk(outbound, nextNext)` (~2261) uses only edge/body headings (no live yaw), and the projection path already latches at arm time (~2326-2327) — so a dispatch-time evaluation is the established, safe pattern. Prove no behavior change with `test_path_tracker_passthrough.cpp`.
- **F4 — do not refactor the tuned ramps in this PR.** The approach branch runs alongside `_useTShapeRampDown`; `PROJECTION_FOLLOW` / `LINE_OVERSHOOT` / seek `OVERSHOOT` stay byte-identical. Unifying is a separate follow-up.
- **Short-hop engage threshold is a documented limit.** Forward hops shorter than ~28-29 cm (axle frame) run flat — see the short-hop Decision. This respects the user's confirmed flat-fallback; a reduced-cruise/triangle fallback is deferred and is the ADR's trigger-to-revisit.
- **Curvy-follow bias is structural.** Encoder distance on a curved follow reads path length, not chord, so the prediction is biased slightly long — the pre-prediction margin + bounded overrun absorb it; keep `LINE_APPROACH_MAX_MARGIN_CM` generous (15, the seek's proven value).
- **Seek arm-window note (not a change).** On the seek, the floor is reached before the ARMED transition because the offset (12.02) exceeds the arm window (10) — fine, and it tightens the detect-debounce distance spread. Comment, not code.
- **`Drive.cpp` IS host-linked** in the PathTracker integration ctest targets (ADR-030, issue #106) — T7's seek wiring has integration coverage; the policy adds isolated unit + cross-language coverage. (The "Drive.cpp not host-compiled" note in `TShapeProfilePolicy.h` refers to *standalone* compilation.)
- **Constant-retune fanout.** New Config values touch four surfaces (host goldens, config-parse pins, behavioral tests, silent sim/runtime mirror) — run the FULL ctest AND full pytest after any value change (drive-constant-retune-fanout gotcha).
- **Bench verification (post-merge).** Re-check B3.5 (nav traversal) and B3.7 (stopAtLine ±2 cm) on the real chassis after this lands — the arrival profile changes where/how the robot stops, and this branch (`test/tune-robot-everything`) is where those are being tuned.
- **Rollback plan:** flip `LINE_APPROACH_RAMP_ENABLED = false` — reproduces today's behavior byte-for-byte without a revert.
