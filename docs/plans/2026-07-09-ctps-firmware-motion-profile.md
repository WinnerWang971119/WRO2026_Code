# CTPS firmware-faithful motion profile + return-to-node + declarePosition (issue #130)

**Goal:** Make CTPS playback mimic the firmware's actual motion — accel→cruise→decel→brake, driven off the same `ConfigDrive.h` constants the robot drives with — instead of a fixed cartoon speed; make raw off-node drives resolve through the firmware's return-to-node path; and make `declarePosition` re-anchors visible. When the firmware is retuned, the playback retunes with it.

**Approach:** Python-side (plus a thin C++ recorder extension and one behavior-identical firmware extraction); `player.js`/`renderer.js` are untouched because the player already does a bounded bisect + linear lerp over timestamped keyframes — so profile *shape* comes purely from emitting **dense keyframes with non-uniform, profile-derived timestamps**. A new `ctps/profile.py` "phase-object kernel" is the single Python source of the T-shape math; it is pinned to the firmware by a **cross-language golden**: the ramp math is extracted from `Drive.cpp` into a host-testable `TShapeProfilePolicy.h`, a host TU dumps golden vectors, and pytest asserts the Python reproduces them bit-for-bit. Raw off-node drives route through the parity-locked sim's existing `operator_move` so the next `goto` renders the firmware's physical return-to-node; `declarePosition` becomes a first-class recorded trace op applied to a new `LocalizationSim.declare_position`.

## Scope

- **In scope:**
  - **Motion profile (gap item 2 / issue body):** each drive/line segment renders the firmware T-shape (accel→cruise→decel), sampled into dense keyframes whose timestamps follow the profile; the stop is visible (ramp-down → deadband-floor crawl → active brake); turns/pivots render their firmware ramp shape (reuse the existing `_build_pivot_arc` arc keyframes + `pivots` overlay, re-time the sub-keyframes). Playback responds to Config changes (bump `TSHAPE_ACCEL_PWM_PER_CM` / cruise PWM → shape changes) with no CTPS-only tuning beyond a single visual scale.
  - **Return-to-node (gap item 1):** route raw off-node `drive` commands through `PathTrackerSim.operator_move(...)` so the operator offset integrates; the next `goto`'s `step_through` already yields the physical return-to-node prologue, which CTPS renders (now profiled). Keep a residual drift indicator only for a raw move at trace end (nothing re-anchors it) or an over-cap off-map fault.
  - **declarePosition visible (gap item 3):** record `Mission.declarePosition(name)` as a command-stream op (`CTPS_RECORD` hook in `Mission.cpp`, `recordDeclarePosition`), add `LocalizationSim.declare_position(node)` (change node, clear offset, preserve heading calibration), apply it during playback, surface it as a localization-lane marker.
  - **Per-command resolved speed:** the recorder mock captures the currently-dropped `.setPower` / `.setPreset` / `.setStartSpeed` / `.setEndSpeed` / `.tShape` / `.flat` / `.setBrake` knobs (stays thin — captures facts, doesn't apply defaults); Python resolves the effective `(cruise, start, end, profiled)` per move against parsed Config defaults + the firmware default rules. goto-expanded hops (no recorded speed) use Config defaults per primitive kind.
  - **Firmware parity extraction:** extract the distance-domain T-shape ramp/triangle/half-cap math + speed-at-fraction from `Drive.cpp` into a pure header-only `TShapeProfilePolicy.h`; `Drive.cpp` delegates (behavior-identical). A host golden-vector dump + pytest parity test pin `profile.py` to it.
  - **Timeline:** repurpose the existing `drift` lane into a broader **localization** lane (same lane id, relabelled), carrying drift + return-to-node + declarePosition markers.
  - **"No drift, 100%":** verify a representative clean mission lands drift ≈ 0 at every `goto` arrival across all hop types; investigate and close any residual (do not loosen the threshold). Boundary-pose invariant keeps landing poses byte-unchanged.
  - **Docs:** amend ADR-033 (drop its "playback is constant-velocity" clause), add ADR-058, note the additive ADR-051 recorder change.
- **Out of scope:**
  - Predicting exact competition wall-clock run time (explicit non-goal; absolute time is NOT an acceptance gate).
  - Wheel slip, battery sag, IMU noise — this is a kinematic-profile preview, not a physics sim.
  - **Gap item 4 (measured termination outcomes):** `stopAtLine` / `wallStop` / `lineFollowDist` keep the nominal-`cm` host estimate. Replacing it with measured LRS/firmware outcomes stays deferred to LRS (#23). (Not selected for this plan.)
  - Any change to `player.js` / `renderer.js` seek/lerp engine, or a Track-schema easing channel (both consultants rejected moving profile math into the browser).
  - Extracting the *turn* (TurnGyro) profile into a firmware policy header — the turn shape is mirrored in Python and pinned with a golden vector; only the *drive* T-shape math is extracted from `Drive.cpp`.

## Decisions

- **Profile math lives in Python (`ctps/profile.py`), not `player.js`.** Both consultants (Fable, Codex) independently endorsed this and rejected an in-browser easing channel: dense keyframes need zero player changes, and the sim owner keeps single-source parity.
- **Closed-form phase durations, not forward-Euler `dt = ds/(k·pwm)` as the clock.** Both consultants independently flagged this as the core defect: `dt = ds/(k·pwm(s))` with `endSpeed=0` diverges (infinite crawl at every stop) and is step-size sensitive, which would break the "does duration change when Config changes" acceptance test. Instead each ramp phase (linear PWM in distance) gets a closed-form duration; the deadband floor is modeled (`v(s) = k·max(pwm(s), PWM_MIN_MAGNITUDE)`); the stop is a floor-crawl segment then a terminal brake keyframe. Dense keyframes are display-only, pinned so the last sub-keyframe of each primitive equals the sim snapshot pose exactly.
- **Parity mechanism: extract `TShapeProfilePolicy.h` from `Drive.cpp` + cross-language golden vectors** (user pick; Fable-recommended). The firmware itself is the golden source, so a future retune regenerates the goldens and CI fails if the Python desyncs. Behavior-identical extraction guarded by arduino-cli compile + host ctest; also gives the firmware profile the host-test coverage it lacks today. (Codex was willing to freeze `Drive.cpp` with hand-derived goldens; user chose the stronger executable tether.)
- **`<5cm` moves render flat cruise, not triangular** (user pick; firmware-faithful). `Drive.h:1798` enqueues a profiled move only when `_tShape && accelValid && _cm >= TSHAPE_MIN_MOVE_CM`; below 5 cm it degrades to a flat `DRIVE_DIST`. Regimes: **flat cruise `<5cm`**, **triangular `[5cm, accel+decel)`**, **full trapezoid `≥ accel+decel`**. The issue body's AC wording ("short moves render triangular") is corrected here.
- **`.setBrake(false)` defaults `endSpeed` to cruise, not 0** (`ConfigDrive.h:572-575`). The Python default resolver must apply this, or every no-brake/chained move renders a phantom decel-to-zero.
- **Per-primitive-kind cruise for goto hops.** goto hops carry no recorded speed, so each primitive kind sources its cruise from Config: blind/dead-reckon → Drive preset T-shape (default `SPEED_NORMAL`); LINE follow phase → `ConfigLine::CRUISE_SPEED`; distance-gated stops / LINE_OVERSHOOT roll → ramp cruise→`PWM_MIN_MAGNITUDE` then brake; `wallStop` → constant `WALL_STOP_PUSH_PWM` crawl + `WALL_STOP_BACK_OFF_CM` reverse; turns → two-stage `TURN_CRUISE_PWM`/`TURN_SLOW_SPEED_PWM`.
- **One CTPS-only visual constant.** A single `SCALE_CM_S_PER_PWM` (cm/s per PWM unit) tuned so a default-cruise move renders at ≈ today's 30 cm/s (preserving the current feel; changeable later). Turn angular rate is *derived* from that same scale + `TRACK_CM` (`ω = (2·k·pwm / TRACK_CM)·180/π` in-place, half for a pivot) rather than a second independent constant — `SIM_DRIVE_CM_S` / `SIM_TURN_DEG_S` are removed.
- **Return-to-node reuses the parity-locked sim.** `operator_move` + the existing `step_through` return prologue are the firmware-faithful path; CTPS stops maintaining its own drift model. The offset must integrate only through `accumulate_operator_move` and never through the CTPS-only pivot-arc chassis nudge.
- **declarePosition is a first-class localization op**, semantically distinct from return-to-node (operator-declared belief reset vs. physical correction before `goTo`).
- **Bounded, fail-loud config parser.** Extend `config_parse` for `constexpr int16_t/uint8_t` + one-level identifier RHS (`TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE`, `CRUISE_SPEED = SPEED_NORMAL`) resolved across `ConfigDrive.h` / `ConfigGlobal.h` / `ConfigLine.h` only; fail loud on any expression or chain (issue-#44 no-stale-fallback policy).
- **No UI/UX design gate.** This changes motion *timing* and relabels one existing lane; it introduces no new visual components, layout, or design language, so the front-end-design gate does not apply.
- **`TURN_SLOW_SPEED_PWM = 20 < PWM_MIN_MAGNITUDE = 50` — parity to verify:** confirm whether the firmware turn emit path snaps it to the deadband floor (like the drive emitters) and mirror whichever PWM the firmware actually commands.

## Acceptance Criteria

- [ ] AC1 — Each drive/line segment renders an accel→cruise→decel speed shape sampled from the firmware T-shape: flat cruise for `<TSHAPE_MIN_MOVE_CM` (5 cm), triangular for `[5, accel+decel)`, full trapezoid for `≥ accel+decel`.
- [ ] AC2 — The stop is visible: a profiled move ramps cruise→`PWM_MIN_MAGNITUDE`, crawls at the floor, then brakes to a terminal keyframe — it does not snap to rest at the boundary, and does not render an infinite asymptotic crawl.
- [ ] AC3 — Turns and pivots render their firmware ramp shape (two-stage `TURN_CRUISE_PWM`→`TURN_SLOW_SPEED_PWM`), reusing the existing pivot arc keyframes + `pivots` overlay with re-timed sub-keyframes.
- [ ] AC4 — Playback is a function of the Config constants: doubling `TSHAPE_ACCEL_PWM_PER_CM` halves the ramp durations, and raising cruise PWM speeds the render up — verified by a config-perturbation test — with no edit to CTPS-only tuning beyond `SCALE_CM_S_PER_PWM`.
- [ ] AC5 — No drift, 100%: a representative clean mission lands the chassis on each `goto` node (drift ≈ 0, float-noise only) across LINE / BLIND / PROJECTION / STOP_AT_LINE / pivot / reverse hops; any residual is investigated and closed, not masked by a loose threshold.
- [ ] AC6 — Absolute run time is NOT gated; relative dynamics (speed-up / slow-down / stop) are.
- [ ] AC7 — A raw off-node drive routes through `operator_move`; the following `goto` renders a physical return-to-node maneuver (line-seek or dead-reckon per the sim), with a brief return marker; a residual drift indicator appears only for a raw move at trace end or an over-cap off-map fault (surfaced as a fault event on the `FAULT` gate).
- [ ] AC8 — `Mission.declarePosition("N")` is recorded, applied to the sim (current node → N, offset cleared, heading calibration preserved), and shown as a localization-lane marker; a `declarePosition`-free trace is byte-unchanged.
- [ ] AC9 — `profile.py` reproduces the host-generated `TShapeProfilePolicy` golden vectors bit-for-bit; the parity test is CI-enforced.
- [ ] AC10 — `player.js` / `renderer.js` are unchanged; every primitive's final sub-keyframe byte-equals the sim snapshot pose (geometry + drift parity preserved); keyframe timestamps are strictly monotonic non-decreasing.
- [ ] AC11 — The Arduino firmware build is byte-identical for the sketch target (`CTPS_RECORD` undefined; the `Drive.cpp` extraction is behavior-identical); `arduino-cli compile`, the host ctest harness, and `uv run --extra dev pytest tools/path_planner/tests/` all pass.

## Contracts & Interfaces

Single source of truth for every cross-task seam. A task that touches a seam conforms to the exact name/shape here.

### Trace `drive` op fields (cross-language: recorder ↔ trace_loader ↔ track_builder)

The `drive` op JSON gains these **optional, additive** fields (absent → Python resolves the firmware default). Existing fields (`method`, `cm`, `deg`, `mode`, `heading`, `reverse`) are unchanged.

```
drive: {
  method, cm, deg, mode, heading, reverse,      # existing
  power:      int|null,    # .setPower override (PWM); null = use preset/default
  preset:     "slow"|"normal"|"fast"|null,  # .setPreset; null = default NORMAL
  start_speed:int|null,    # .setStartSpeed; null = TSHAPE_START_SPEED
  end_speed:  int|null,    # .setEndSpeed; null = default per brake rule
  profiled:   bool|null,   # true=.tShape, false=.flat; null = builder default (tShape on for driveDist)
  brake:      bool|null     # .setBrake; null = default true
}
```

- Owner: **T8** (recorder emits) + **T9** (trace_loader validates/retains). Consumers: **T10/T11** (track_builder), **T5** (profile resolver).
- Parsing follows the existing strict-type guards in `trace_loader.py` (reject non-numeric/`bool`-as-int; warn + treat as unset).

### `declare_position` trace op (cross-language: Mission.cpp ↔ trace_loader ↔ track_builder)

```
{ op: "declare_position", node: "<name>" }
```

- Owner: **T8** (`recordDeclarePosition(name)` + JSON) + **T9** (loader). Consumers: **T11** (track_builder → `sim.localization.declare_position`).

### `ctps::DriveMoveParams` (C++ struct, `ctps_record.h`)

Extends the existing `{cm, mode, hasHeading, heading, reverse}` with `power`/`preset`/`startSpeed`/`endSpeed`/`profiled`/`brake` + per-field "was-set" sentinels. Owner: **T8**. Consumers: the six recording builders in `ctps_mocks/Drive.h`, the `ctps_record.cpp` JSON serializer, the ADR-051 conformance TU.

### `TShapeProfilePolicy.h` (C++ header-only, pure)

Distance-domain, Arduino-free, POD in / POD out (mirrors the `*Policy.h` pattern). Exact signatures set by **T2**:

```cpp
struct TShapeRamps { float accelCm; float decelCm; int16_t peakPwm; };
// Mirrors _execProfileMove's resolveRamp + triangle fallback + half-cap.
TShapeRamps resolveTShapeRamps(float totalCm, int16_t cruise, int16_t startSpd,
                               int16_t endSpd, float accel, float rampMin,
                               float rampMax, int16_t noRampEps);
// Mirrors _profileSpeedAt (distance domain): PWM at a traveled fraction.
int16_t tShapeSpeedAt(float traveledCm, float totalCm, const TShapeRamps& r,
                      int16_t startSpd, int16_t endSpd);
```

- Owner: **T2**. Consumers: `Drive.cpp` (delegates), the host golden-dump TU (**T3**). The Python mirror in `profile.py` (**T5**) is pinned to it via the goldens (**T3**/**T6**), not a code dependency.

### `profile.py` public API (Python)

Owner: **T5**. Consumer: **T10** (track_builder).

```python
resolve_drive_profile(dist_cm, *, cruise_pwm, start_pwm, end_pwm, profiled) -> DriveProfile
    # DriveProfile exposes phase boundaries (accel_cm, cruise_cm, decel_cm, floor tail),
    # peak_pwm, pwm_at_s(s), and phase durations via closed-form ds/(k·pwm) integration.
resolve_turn_profile(theta_deg, *, is_pivot) -> TurnProfile
    # Two-stage cruise/slow-zone shape; angular rate derived from SCALE + TRACK_CM.
sample_drive(profile, start_pose, end_pose, t0) -> list[keyframe]   # dense near ramps, sparse on cruise
sample_turn(profile, ...) -> list[keyframe]
resolve_effective_speed(cmd, method) -> (cruise, start, end, profiled)  # applies Config defaults + overrides
```

### `LocalizationSim.declare_position` (Python, `sim/runtime.py`)

```python
def declare_position(self, node_name: str) -> None
    # Mirror of firmware Localization.declarePosition: set current node, clear offset,
    # preserve heading calibration (_yaw offset). Node-name validated against NODES.
```

- Owner: **T7**. Consumer: **T11**.

### CTPS visual/timing config (`ctps/config.py`)

- `SCALE_CM_S_PER_PWM: float` — the single visual scale. Removes `SIM_DRIVE_CM_S` / `SIM_TURN_DEG_S`.
- Sampling knobs: ramp-sample step / error tolerance + keyframe-count ceiling; `ARC_STEP_DEG`, `DRIFT_THRESHOLD_CM` (kept as a display constant per the issue), `ROBOT_FRAME_CM`, cargo constants unchanged.
- Owner: **T4**. Consumers: **T5**, **T11**.

### Localization-lane event types (Python emit ↔ JS render)

`track_builder` emits localization-lane events with `type` ∈ {`drift_warn` (residual only), `return_to_node`, `declare_position`}. The player's repurposed lane renders all three. Owner of the emit set: **T11**; owner of the render: **T12**. Both point at this list.

### Golden-vector JSON (host ↔ pytest)

`tests/fixtures/tshape_profile_goldens.json` — `[{dist, cruise, start, end, accel, expect:{accelCm, decelCm, peakPwm, pwm_at:[...]}}...]`. Owner: **T3** (host TU generates). Consumer: **T6**.

## Data Model / trace additions

- `drive` op: additive optional fields (above). A trace with none of them is byte-identical to today.
- `declare_position` op: new op kind.
- `Track` schema (`track_builder.Track`): **unchanged shape** — still `{schema_version, field, duration_s, keyframes, events, pivots?}`. Densification only changes the *number* and *timestamps* of keyframes; localization events reuse the events list. `schema_version` stays `1` (additive).

## Firmware extraction (T2) — contract

- Move the pure distance-domain math out of `DriveClass::_execProfileMove` (ramp resolution + triangle fallback + half-cap + peak) and `_profileSpeedAt` (distance branch) into `TShapeProfilePolicy.h`. `Drive.cpp` calls the policy; the time-domain branch, encoder conversions, and state-machine wiring stay in `Drive.cpp`.
- **Behavior-identical:** no change to emitted PWM for any move. Verified by (a) `arduino-cli compile` green, (b) the host ctest harness green, (c) the new host golden dump matching hand-checked values at representative `(dist, cruise, start, end)` points.
- The Arduino sketch build must stay byte-identical (the policy is header-only, included where `_execProfileMove` lived).

## Error Handling

- **Config parse failure** (missing/unparseable constant, unresolvable symbolic RHS, an expression the bounded resolver refuses): raise loudly (issue-#44 policy) — no stale-literal fallback. Surfaces at CTPS build time with the constant name + file.
- **Over-cap off-map excursion:** the sim raises `OffMapFaultError` under the `FAULT` gate — track_builder surfaces it as a fault event and ends the track (like `UnreachableGoalError`); under `BEST_EFFORT` it renders the capped return with a marker. CTPS mirrors whatever `Config::OFF_MAP_GATE` / the sim resolves — no CTPS-specific policy.
- **Raw move at trace end (no following goto):** offset never re-anchors; keep a residual drift indicator measuring chassis-vs-node at trace end.
- **`declarePosition` to an unknown node:** loader/sim validate the name; an invalid node warns and is skipped (mirrors the loader's strict-field pattern), not a hard crash.
- **Degenerate profile inputs** (zero/negative distance, cruise 0, accel ≤ 0): `profile.py` mirrors the firmware defensive clamps (`_execProfileMove` floors negatives, bails on `accel ≤ 0`); a zero-length segment emits a single keyframe (no divide-by-zero in the closed-form duration).

## Testing Strategy

**Levels:** Python unit (profile kernel, config parse, sim), Python integration (track_builder), host C++ ctest (policy + recorder conformance), manual eyeball (rendered player), regression (drift, byte-identity).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `profile.py` vs host golden vectors | Unit (parity) | Ramp distances, peak, and `pwm_at` reproduce the goldens bit-for-bit (AC9). |
| TC2 | Config perturbation: double `TSHAPE_ACCEL_PWM_PER_CM` (temp header via `config_path`) | Unit (property) | Resolved ramp durations halve; render duration changes (AC4). |
| TC3 | Regime boundaries: 3 cm / 12 cm / 40 cm moves | Unit | 3 cm → flat cruise (no ramp); 12 cm → triangular (peak < cruise, no plateau); 40 cm → full trapezoid (AC1). |
| TC4 | Stop shape at `endSpeed=0` | Unit | Finite duration; a floor-crawl segment at `PWM_MIN_MAGNITUDE` then a terminal brake keyframe; no asymptotic crawl (AC2). |
| TC5 | `.setBrake(false)` move | Unit | Default `endSpeed` resolves to cruise → no phantom decel-to-zero (default rule). |
| TC6 | Turn / pivot re-timing | Unit | Sub-keyframes follow the two-stage turn shape; arc keyframes + `pivots` overlay preserved; `SLOW_SPEED_PWM` snap-parity matched (AC3). |
| TC7 | Boundary-pose invariant | Integration | For every primitive, the last sub-keyframe byte-equals a parallel `step_through` snapshot; timestamps strictly monotonic (AC10). |
| TC8 | Clean-mission drift sweep (all hop types) | Integration | Chassis-vs-node offset ≈ 0 (float noise) at every `goto` arrival; regression-locked (AC5). |
| TC9 | Raw drive → next goto | Integration | Raw drive integrates offset; next goto emits + renders the return-to-node maneuver; residual indicator absent when a goto follows (AC7). |
| TC10 | Raw drive at trace end / over-cap fault | Integration | Trace-end raw move → residual drift indicator; over-cap under `FAULT` → fault event ends track (AC7). |
| TC11 | `declarePosition` op | Integration | Recorded op applies `declare_position` (node change + offset clear, heading preserved); localization-lane marker emitted; declarePosition-free trace byte-identical (AC8). |
| TC12 | Recorder captures per-command knobs | Host ctest / superset | `.setPower/.setPreset/.setStartSpeed/.setEndSpeed/.tShape/.flat/.setBrake` are captured + serialized; ADR-051 conformance + Python name-superset pass (AC11). |
| TC13 | `config_parse` int + symbolic RHS | Unit | Parses `constexpr int16_t`, resolves `PWM_MIN_MAGNITUDE` / `CRUISE_SPEED=SPEED_NORMAL`; fails loud on an expression/chain (AC4 support). |
| TC14 | `TShapeProfilePolicy` host test | Host ctest | Policy ramp/triangle/peak/speed-at match hand-checked values; Drive.cpp still compiles + drives identically (AC11). |
| TC15 | Firmware byte-identity | Host ctest / arduino-cli | Sketch build unchanged; `_execProfileMove` behavior identical post-extraction (AC11). |
| TC16 | Rendered player eyeball | Manual | Serve `output/` over `http.server`, drive Chrome MCP: accel/decel/stop visible, return maneuver renders, declarePosition marker + relabelled localization lane present (AC1-3, AC7-8). |

**Test data:** existing `tests/fixtures/ctps_*.json` traces (still valid — new fields additive); new fixtures for per-command speed, `declare_position`, and a raw-drive→goto return. Golden vectors generated by the host TU. `config_parse` tests use a temp header via the existing `config_path` seam.

**Run commands:** `uv run --extra dev pytest tools/path_planner/tests/` ; `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure` ; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Extend `config_parse` for ints + symbolic RHS | — | med | `tools/path_planner/config_parse.py`, `tools/path_planner/tests/test_config_parse.py` (or existing) | Add `parse_config_int` (`constexpr int16_t/uint8_t`) and one-level identifier-RHS resolution (`TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE`, `CRUISE_SPEED = SPEED_NORMAL`) across `ConfigDrive.h`/`ConfigGlobal.h`/`ConfigLine.h`; fail loud on expressions/chains. Satisfies TC13. |
| T2 | Extract `TShapeProfilePolicy.h`; `Drive.cpp` delegates | — | xHigh | `WRO2026/src/drive/TShapeProfilePolicy.h` (new), `WRO2026/src/drive/Drive.cpp`, `WRO2026/src/drive/Drive.h` | Move the pure distance-domain ramp/triangle/half-cap + speed-at math out of `_execProfileMove`/`_profileSpeedAt` into a header-only pure policy (see Contracts). Behavior-identical; sketch build byte-identical. Satisfies TC14/TC15, AC11. |
| T3 | Host golden-vector dump + ctest + goldens JSON | T2 | med | `WRO2026/test/tshape_profile_dump/*` (new), `WRO2026/test/CMakeLists.txt`, `tools/path_planner/tests/fixtures/tshape_profile_goldens.json` | Host TU that includes `TShapeProfilePolicy.h` + real Config and dumps `(dist,cruise,start,end,accel)→(accelCm,decelCm,peakPwm,pwm_at[])` vectors to the goldens JSON; wire a ctest target. Emits the fixture consumed by TC1. |
| T4 | CTPS visual/timing config | — | low | `tools/path_planner/ctps/config.py` | Add `SCALE_CM_S_PER_PWM` (tuned so default cruise ≈ 30 cm/s) + sampling tolerance/ceiling; remove `SIM_DRIVE_CM_S`/`SIM_TURN_DEG_S`; keep `DRIFT_THRESHOLD_CM` (display), `ARC_STEP_DEG`, `ROBOT_FRAME_CM`, cargo consts. Satisfies the Contracts config seam. |
| T5 | `ctps/profile.py` phase-object kernel | T1, T4 | high | `tools/path_planner/ctps/profile.py` (new) | Parse Config via T1; `resolve_drive_profile` mirrors the T-shape (regime boundaries incl. `<5cm` flat, brake rule), closed-form phase durations with the deadband floor; `resolve_turn_profile` (two-stage, ω from scale+`TRACK_CM`); `sample_drive`/`sample_turn` (dense near ramps, sparse cruise); `resolve_effective_speed` (Config defaults + overrides). Public API per Contracts. |
| T6 | Profile parity + shape + perturbation tests | T3, T5 | med | `tools/path_planner/tests/test_ctps_profile.py` (new) | TC1 (vs goldens, bit-for-bit), TC2 (perturbation), TC3 (regimes), TC4 (stop shape), TC5 (no-brake). |
| T7 | `LocalizationSim.declare_position` | — | med | `tools/path_planner/sim/runtime.py`, `tools/path_planner/tests/` (sim test) | Add `declare_position(node_name)` mirroring firmware (set node, clear offset, preserve heading); validate node name. Sim stays timing-free. Supports TC11. |
| T8 | Recorder: per-command speed + `declarePosition` | — | med | `WRO2026/test/ctps_mocks/ctps_record.h`, `ctps_record.cpp`, `WRO2026/test/ctps_mocks/Drive.h`, `WRO2026/test/ctps_mocks/Localization.h`, `WRO2026/src/mission/Mission.cpp`, `WRO2026/test/ctps_mocks/test_ctps_mock_conformance.cpp` | Extend `DriveMoveParams` + the six builder setters to capture the dropped knobs (thin — facts only); serialize to the additive `drive` fields; add `recordDeclarePosition(name)` + a `#ifdef CTPS_RECORD` hook in `Mission.cpp::declarePosition`; update the conformance TU. Satisfies TC12. Keep the sketch build byte-identical (`CTPS_RECORD`-guarded). |
| T9 | `trace_loader`: new drive fields + `declare_position` op | — | med | `tools/path_planner/ctps/trace_loader.py`, `tools/path_planner/tests/test_ctps_trace_loader.py` | Add the additive `drive` fields (strict type guards, absent→None) + the `declare_position` op to the `Command` schema/validation. Conforms to the Contracts JSON seam (coordinate with T8). |
| T10 | track_builder: profile densification | T5, T9 | high | `tools/path_planner/ctps/track_builder.py` | Replace `_pose_delta_duration`/`append_geometry_keyframe` flat timing with `profile.py`-driven densification for drive/line segments and turns; re-time the `_build_pivot_arc` sub-keyframes with the turn profile; last sub-keyframe == snapshot pose exactly. goto hops use per-kind Config cruise; raw drives use resolved recorded speed. Satisfies AC1-3, AC6, AC10 (partial). |
| T11 | track_builder: return-to-node + declarePosition + localization events | T10, T7 | high | `tools/path_planner/ctps/track_builder.py` | Route raw `drive` moves through `sim.operator_move` (incl. `turnToYaw`→`operator_move(0, turn_deg=wrap_pm180(...))` and the gyro two-phase turn); render the `step_through` return prologue (profiled); keep residual drift indicator only for trace-end/over-cap; apply `declare_position` op → `sim.localization.declare_position` + a localization-lane marker; keep pivot-arc chassis nudge isolated from the offset integrator. Satisfies AC7, AC8. |
| T12 | Player: repurpose drift lane → localization | — | med | `tools/path_planner/ctps/assets/timeline.js`, `tools/path_planner/ctps/player_html.py`, `tools/path_planner/ctps/assets/components.css`, `tools/path_planner/ctps/assets/selection.js`, `tools/path_planner/tests/test_ctps_player_html.py` | Keep the lane **id** `drift`, relabel to "Localization", broaden its builder to render `return_to_node` + `declare_position` markers alongside `drift_warn`; update any frozen `PLAYER_META` test literals. Conforms to the localization-lane event-type seam (T11). |
| T13 | Rebaseline track_builder tests + boundary invariant | T10, T11 | med | `tools/path_planner/tests/test_ctps_track_builder.py`, `tools/path_planner/tests/fixtures/ctps_*.json` | Replace the 1:1 keyframe↔primitive assertions with the densified contract; add TC7 (boundary-pose invariant vs `step_through`), TC8-scaffold, TC9-TC11 track-level; regenerate track goldens after the invariant proves poses unchanged. |
| T14 | E2E + cross-platform + mock-superset + output goldens | T8, T9, T11, T13 | med | `tools/path_planner/tests/test_ctps_end_to_end.py`, `tools/path_planner/tests/test_ctps_cross_platform.py`, `tools/path_planner/tests/test_ctps_mock_superset.py`, `tools/path_planner/tests/fixtures/*` | End-to-end dump→load→build→render with the new fields; superset check for the new captured setters; regenerate output/HTML goldens. Serialize after T13 to avoid fixture races. |
| T15 | Drift "100%" investigation + regression | T11 | high | `tools/path_planner/tests/test_ctps_drift.py` (new) + any fix in `track_builder.py`/`profile.py` | Run a representative clean mission through CTPS; assert drift ≈ 0 at every `goto` arrival across LINE/BLIND/PROJECTION/STOP_AT_LINE/pivot/reverse; if a residual > float-noise appears, root-cause and fix (do not loosen the threshold). Locks TC8 / AC5. |
| T16 | Eyeball rendered player | T11, T12 | low | (manual; serve `output/` over `http.server` + Chrome MCP) | Verify motion shape (accel/decel/stop visible), return maneuver, declarePosition marker, relabelled localization lane. TC16. Chrome MCP refuses `file://` — serve over `127.0.0.1`. |
| T17 | ADRs + index | — | low | `docs/decisions/033-ctps-derived-mission-trace.md`, `docs/decisions/058-ctps-firmware-motion-profile.md` (new), `docs/decisions/051-ctps-mock-conformance-guard.md`, `DECISIONS.md` | Amend ADR-033 (drop "playback is constant-velocity", link 058); write ADR-058 (Config-derived phase-object profile + closed-form durations + deadband floor + dense-keyframes-display-only + extracted `TShapeProfilePolicy` + return-to-node via `operator_move` + `declarePosition` op + drift→localization lane); note the additive ADR-051 recorder change; update the index + status table. |

**Parallelism:** Wave 1 (no blockers): T1, T2, T4, T7, T8, T9, T12, T17. Wave 2: T3(T2), T5(T1,T4). Wave 3: T6(T3,T5), T10(T5,T9). Wave 4: T11(T10,T7). Wave 5: T13(T10,T11), T15(T11), T16(T11,T12). Wave 6: T14(T13...). T10→T11→T13 serialize on `track_builder.py`.

## Notes for Implementer

- **Both consultants converged** on: closed-form phase durations (not forward-Euler sampling as the clock), deadband-floor modeling, dense-keyframes-display-only, thin recorder + Python default resolution, `operator_move` for raw drives, `declarePosition` as a first-class op. Treat those as settled.
- **The stop is the trap.** `dt = ds/(k·pwm(s))` with `endSpeed=0` diverges — the integral of `ds/v` near a linear zero is logarithmically infinite. Model `v(s) = k·max(pwm(s), PWM_MIN_MAGNITUDE)`: the decel ramp goes peak→50, then a constant-50 floor-crawl from the 50-crossing to move end, then an instantaneous brake (a terminal keyframe). Emit keyframes at the exact phase boundaries + a few interior ramp samples.
- **Regime boundaries are subtle and firmware-verified.** `<5cm` = flat cruise (`Drive.h:1798` gates `PROFILE_DIST` on `_cm >= TSHAPE_MIN_MOVE_CM`); `[5, accel+decel)` = triangular (peak = `(start+end+cruise)/3` clamped ≤ cruise); `≥ accel+decel` = full trapezoid. `.setBrake(false)` → default `endSpeed = cruise` (`ConfigDrive.h:572-575`).
- **`TURN_SLOW_SPEED_PWM = 20 < PWM_MIN_MAGNITUDE = 50`** — verify whether the firmware turn emit path snaps it to the deadband floor (the drive emitter does via `deadbandSnap`); mirror whatever PWM the robot actually commands, not the raw constant.
- **Isolate the offset integrator from the pivot-arc nudge.** The existing `_build_pivot_arc` chassis nudge writes `chassis.x/y` directly and must NOT feed `accumulate_operator_move` (it doesn't today — the write never reaches `LocalizationSim`). Keep it that way; TC7/TC8 are the tripwires.
- **Return-to-node primitives to handle in track_builder:** the prologue yields `ACT_TURN_REL` (pre-edge turn) + `ACT_RETURN_FORWARD` (dead-reckon) or `ACT_RETURN_STOP_AT_LINE` (line-seek) — all before the planned walk. `find_pivot_turn(prev_node=None, ...)` returns None on the return's turn (no predecessor), so it renders as a plain turn — no spurious arc. Verify.
- **`DRIFT_THRESHOLD_CM`** is now essentially a display constant (the CTPS gate was already removed; it's referenced in `selection.js` as a UI literal). The issue asks to keep/verify it — keep it, and confirm a clean mission never even triggers the residual indicator.
- **Trace fixtures stay valid** — all new `drive` fields are optional; only add new fixtures for the new behaviors.
- **Rollback:** the whole change is additive/behavior-preserving except the `Drive.cpp` extraction (T2). If T2 regresses motion, revert just T2 + fall back to hand-derived golden vectors in `test_ctps_profile.py` (Codex's alternative) and file the extraction as a follow-up — the Python side is unaffected.
- **Eyeballing:** the timeline-lane change and the motion shape need a rendered-page check — serve `tools/path_planner/output/` over `python -m http.server` and drive the Chrome MCP extension (it refuses `file://`).
