# Pass-Through Collinear Intersections Plan

**Issue:** [#19](https://github.com/wangdiego1119/WRO2026_Code/issues/19) — PathTracker:
pass straight through intersections without stopping when next edge is collinear.

**Goal:** When the next-hop edge after an intersection node `B` is collinear with the inbound
edge (heading delta ≤ `Config::NAV_TURN_TOLERANCE_DEG`), traverse `B` without braking. Keep the
chassis at cruise PWM through the 5 cm sensor-to-axle overshoot, then hand off live to the
next edge — re-using LineFollower if the next edge is LINE / PROJECTION, or releasing to
`Drive.driveDist(...).setStartSpeed(cruise)` if the next edge is BLIND.

**Approach:** Three coordinated changes plus a deliberate project-wide overshoot-semantics
change (owned by ADR-023, refining ADR-018).

1. **LineFollower** gains two builder knobs:
   - `setOvershootMode(bool)` — when `true`, the controller dead-reckons symmetric cruise PWM
     whenever the line goes offline, instead of firing `Config::LOST_LINE_ACTION`. Applied for
     the entire 5 cm overshoot window across **every** caller of LineFollower (not just
     pass-through — same change in both pass-through and stop-and-go intersections). This is a
     project-wide behavior change to the distance-gated stop semantics defined in ADR-018;
     **ADR-023 owns this design decision explicitly** and is cross-linked as "Refines ADR-018".
   - `setBrakeOnGate(bool)` — when `false`, the distance-gate trip latches `_distanceReached`
     and clears the active gate flag, but does NOT self-disable the controller and does NOT
     call `Drive.stop`. Lets PathTracker poll `isDistanceReached()` while the controller keeps
     issuing `Drive.setSpeed` at cruise.

2. **PathTracker** peeks the next-hop at `Intersection.isAtIntersection()` (still in
   `LINE_FOLLOW`), decides pass-through eligibility (`_passThrough` flag), and arms the
   overshoot accordingly. The `LINE_OVERSHOOT` state's gate-trip handler branches on
   `_passThrough`. `_dispatchEdgeAfterTurn` learns to:
   - LINE next: same call as today; the already-enabled controller path skips ramp-up.
   - PROJECTION next: same call as today plus `.setOvershootMode(false)` to flip back to P+D.
   - BLIND next during pass-through: disable LineFollower with `.setBrake(false)` first, then
     `Drive.driveDist(...).setStartSpeed(currentCruise)` so the velocity profile starts at
     cruise instead of `Config::TSHAPE_START_SPEED` (the deadband floor).

3. **Sim** (`tools/path_planner/sim/runtime.py`) introduces an `ACT_LINE_PASS` primitive,
   emitted in place of `ACT_LINE_OVERSHOOT` whenever the next-hop edge from the intersected
   node is collinear with the inbound edge. Position update is identical; the primitive name
   distinguishes pass-through hops for verification + telemetry parity.

## Scope

- **In scope:**
  - Pass-through decision at LINE_OVERSHOOT (inbound edge is LINE; next edge is any type).
  - LineFollower API additions: `setOvershootMode(bool)`, `setBrakeOnGate(bool)`.
  - Change the 5 cm overshoot semantics project-wide: LineFollower dead-reckons symmetric
    cruise (instead of dispatching `Config::LOST_LINE_ACTION`) whenever the line is offline
    during an overshoot, regardless of pass-through state. Owned by ADR-023 (refines ADR-018).
  - PathTracker FSM updates: LINE_FOLLOW peek + LINE_OVERSHOOT branch + dispatch handoffs.
  - Sim `ACT_LINE_PASS` primitive + go_to / step_through emission.
  - Pytest goldens for collinear sequences (LINE→LINE, LINE→PROJECTION, LINE→BLIND).
  - `Config::PATH_PASS_THROUGH_ENABLED` global opt-out.
  - ADR-023 documenting the pass-through rule, the runtime line-presence probe (overshoot
    mode) semantics, and the heading-tolerance contract. Refines ADR-018.
  - Host-compile test harness under `WRO2026/test/` for C++ unit tests covering the
    LineFollower overshoot-mode and `PATH_PASS_THROUGH_ENABLED` toggle behaviors (T10).

- **Out of scope:**
  - Pass-through at PROJECTION exit (PROJECTION_FOLLOW state) — different gate mechanism;
    follow-up issue.
  - Pass-through at BLIND exit (BLIND_FORWARD state) — requires Drive end-speed handoff that
    doesn't exist today; follow-up issue.
  - Ramping cruise between different LineMode/SpeedPreset values — preset snaps mid-stride
    when A→B and B→C have different presets (acceptable velocity discontinuity).
  - Direction reversal at an intersection — `_reverse` differing between edges produces a
    180° heading delta which automatically falls back to stop-and-go via the heading check.
  - Per-call PathTracker.goTo(...).noPassThrough() builder opt-out — Config-level boolean
    only.
  - Hardware verification — documented in T9 below but deferred to Phase 3 bring-up.

## Decisions

- **Heading tolerance:** reuse `Config::NAV_TURN_TOLERANCE_DEG` (0.5°). Aligns with the
  existing turn-or-not decision in `_onPlanNextHop`; both checks fire on the same edges.
- **Eligibility check:** `_nextHop != _destinationNode` is a precondition — never pass through
  the destination node, even if a collinear edge continues past it.
- **Decision placement:** at `Intersection.isAtIntersection()` in `LINE_FOLLOW` (issue's
  recommended approach). Peek `FieldMap::getNextHop(_nextHop, _destinationNode)` and the
  resulting edge heading.
- **Mechanism for line-ends-at-node:** runtime probe via LineFollower (overshoot mode flag
  consults `_array->isOnline()` each tick). No plan-time NodeInfo flag needed; works for any
  geometry including pre-existing stop-and-go overshoots into PROJECTION/BLIND nodes.
- **Preset change between edges:** cruise snaps to the new preset (no inter-preset ramp).
  Justified by: the new preset takes effect on the next `update()` tick; observed velocity
  discontinuity is small relative to the velocity savings from skipping the brake.
- **Opt-out:** `Config::PATH_PASS_THROUGH_ENABLED` boolean only (default `true`). Per-call
  builder opt-out deferred — the Config flag is sufficient for debug runs.
- **Telemetry:** no EventTrace change. There is no "INT" event today; the line-lost / turn-end
  events are unaffected.

## Acceptance Criteria

**Verification boundary:** This PR proves the **sim and FSM emit the correct primitives**
under all relevant conditions. It does NOT prove on-robot velocity behavior — that lives in
the Phase 3 hardware battery (HW1–HW3 below). If a Phase 3 hardware test fails, ADR-023 may
need to be revisited.

- [ ] **AC1 (sim-equivalent, LINE→LINE):** For a 3+-node collinear LINE path in the sim, the
      primitive sequence emits `ACT_LINE_PASS` (not `ACT_LINE_OVERSHOOT`) at every
      intermediate collinear intersection. Final node still emits `ACT_LINE_OVERSHOOT`.
      Verified by TC1.
- [ ] **AC2 (sim-equivalent, turn unchanged):** For a path with a > `NAV_TURN_TOLERANCE_DEG`
      heading delta at an intersection, the sim emits `ACT_LINE_OVERSHOOT` (NOT
      `ACT_LINE_PASS`), followed by `ACT_TURN_REL`. Same as today. Verified by TC2.
- [ ] **AC3 (sim-equivalent, LINE→BLIND pass-through):** When the next-hop edge is BLIND and
      collinear, the sim emits `ACT_LINE_PASS` then `ACT_FORWARD`. C++ host-compile test T10
      confirms `Drive.driveDist(...)` is dispatched with `.setStartSpeed(currentCruise)`, not
      `Config::TSHAPE_START_SPEED`. Verified by TC6 (sim) and T10 (host-compile).
- [ ] **AC4 (sim-equivalent, LINE→PROJECTION pass-through):** When the next-hop edge is
      PROJECTION and collinear, the sim emits `ACT_LINE_PASS` then `ACT_FOLLOW_DIST`. C++
      host-compile test T10 confirms LineFollower stays enabled across the intersection (no
      intermediate disable). Verified by TC5 (sim) and T10 (host-compile).
- [ ] **AC5 (Localization correctness):** `Localization.onIntersection(B)` fires exactly
      once per traversed intersection, at the axle-on-node moment (after the 5 cm overshoot),
      whether pass-through or stop-and-go. Graph state matches today's behavior. Verified by
      TC1 (chassis position equality before/after LINE_PASS == LINE_OVERSHOOT at the same
      node) and T10 (host-compile: assertion that `onIntersection` fires exactly once).
- [ ] **AC6 (host-compile, line-ends-at-node bug fix):** Host-compile unit test T10 asserts
      that with `setOvershootMode(true)`, a synthesized "online → offline" sequence drives
      the controller to dead-reckon symmetric cruise (no `LOST_LINE_ACTION` dispatch) while
      the encoder gate continues to tick; gate trip still fires after the 5 cm distance.
      With `setOvershootMode(false)`, today's `LOST_LINE_ACTION` dispatch is unchanged. The
      end-to-end "line ends ~3 cm short of the node, robot still arrives at axle-on-node"
      claim is HW3 (deferred to Phase 3). T10 does NOT cover real-world debounce timing or
      sensor noise.
- [ ] **AC7a (sim opt-out):** TC8 verifies the Python sim mirror of
      `PATH_PASS_THROUGH_ENABLED = False` emits `ACT_LINE_OVERSHOOT` (legacy behavior) at
      every collinear intersection.
- [ ] **AC7b (C++ opt-out):** Host-compile test T10 has two compile-and-run targets,
      `-DPATH_PASS_THROUGH_ENABLED=1` and `-DPATH_PASS_THROUGH_ENABLED=0`. Each target
      asserts the corresponding primitive sequence on the canonical collinear fixture. Both
      targets added to the Arduino CI workflow. A pure compile-only check is **not
      sufficient** — the flag must be exercised behaviorally on both branches.
- [ ] **AC8 (sim emission):** The sim emits `ACT_LINE_PASS` instead of `ACT_LINE_OVERSHOOT`
      for any LINE-edge intersection whose next-hop edge from that node is collinear with
      the inbound edge. Chassis position update is identical to `ACT_LINE_OVERSHOOT`.
      Verified by TC1, TC5, TC6, TC11.
- [ ] **AC9 (junction-freeze unchanged):** During the 5 cm overshoot, if the line array
      classifier still reports a junction marker (LEFT/RIGHT/T_OR_CROSS), the existing PID
      freeze + symmetric cruise behavior continues to fire (issue #22). Overshoot-mode does
      not contradict junction-freeze; both produce symmetric cruise. Verified by host-compile
      T10 (assertion that `_pidFrozen` flag remains true during synthesized
      junction-marker-visible ticks in overshoot mode).
- [ ] **AC10 (destination node):** The final hop's intersection (where `_nextHop ==
      _destinationNode`) NEVER pass-throughs, even if a collinear edge continues past it.
      Chassis stops at the goal. Verified by TC7 (sim) and T10 (host-compile).
- [ ] **AC11 (PROJECTION/BLIND exit unchanged):** When the inbound edge is PROJECTION or
      BLIND (not LINE), behavior is identical to today. The pass-through mechanism does not
      apply at PROJECTION_FOLLOW / BLIND_FORWARD exits. Verified by TC12 (sim — paths whose
      inbound edge is PROJECTION emit unchanged primitive sequence).

**Deferred hardware verification (Phase 3 — out of scope for this PR):**
- [ ] **HW1 (no visible deceleration):** On a 4-node collinear LINE chain on the real
      chassis, stopwatch + OLED PWM telemetry show no visible deceleration at intermediate
      nodes.
- [ ] **HW2 (turn-at-intersection unchanged):** Chassis stop-turns at non-collinear
      intersections, identical to today's pre-PR behavior.
- [ ] **HW3 (line ends at node during overshoot):** Chassis advances the full 5 cm and lands
      axle-on-node when the line physically ends at the node (e.g. before a BLIND segment),
      confirming the AC6 latent-bug fix in real-world conditions.

If any of HW1–HW3 fails, ADR-023 may need revisiting and either the debounce timing, the
heading tolerance, or the runtime-probe mechanism itself may need adjustment.

## Data Model

No persistent data changes. New transient state:

```cpp
// PathTracker.h — private members
bool _passThrough = false;  ///< Set in LINE_FOLLOW at intersection-detect when the next-hop
                            ///< edge after _nextHop is collinear with the inbound edge.
                            ///< Consumed in LINE_OVERSHOOT and _dispatchEdgeAfterTurn; reset
                            ///< to false on every fresh intersection-detect tick.
```

```cpp
// LineFollower.h — new private members
bool _overshootMode = false;  ///< When true, isOnline()==false branch dead-reckons symmetric
                              ///< cruise instead of firing LOST_LINE_ACTION. Cleared on
                              ///< disable, on gate-trip (with brake), and explicitly by
                              ///< setOvershootMode(false) on a re-enable.
bool _brakeOnGate = true;     ///< When false, gate-trip latches _distanceReached + clears
                              ///< _distanceTargetActive but does NOT call _applyEnableBuilder
                              ///< (no self-disable, no Drive.stop). Default true preserves
                              ///< today's behavior.
```

```cpp
// Config.h — new constant
constexpr bool PATH_PASS_THROUGH_ENABLED = true;
```

```python
# tools/path_planner/sim/runtime.py — new primitive tag
ACT_LINE_PASS: str = "line_pass"
```

## API Contracts

### LineFollower builder additions

```cpp
class LineFollowerEnableBuilder {
    LineFollowerEnableBuilder& setOvershootMode(bool on);
    LineFollowerEnableBuilder& setBrakeOnGate(bool on);
};
```

- `setOvershootMode(true)`: only meaningful with `setEnabled(true)`. Causes `update()` to
  dead-reckon symmetric cruise (deadband-snapped) when `isOnline()` is false, instead of
  dispatching `Config::LOST_LINE_ACTION`. Junction-freeze and online P+D are unaffected.
- `setBrakeOnGate(false)`: only meaningful with `setEnabled(true).setDistanceTarget(cm)`. On
  gate trip, `_distanceReached = true` and `_distanceTargetActive = false`; `_useTShape*` are
  cleared; controller stays enabled and keeps issuing `Drive.setSpeed` at cruise.

### PathTracker FSM transitions (delta from today)

**Peek data sources (T4 must use these exact calls):**

```cpp
const uint8_t cur     = Localization.getCurrentNode();
const Routes::EdgeInfo& inbound = FieldMap::getEdge(cur, _nextHop);

bool passThrough = false;
if (Config::PATH_PASS_THROUGH_ENABLED && _nextHop != _destinationNode)
{
    const uint8_t nextNext = FieldMap::getNextHop(_nextHop, _destinationNode);
    if (nextNext != FieldMap::NODE_NONE && nextNext != _nextHop)
    {
        const Routes::EdgeInfo& next = FieldMap::getEdge(_nextHop, nextNext);
        if (next.type != Routes::EdgeType::NONE)
        {
            // Wrap-normalize the heading delta — reuses the existing pattern from
            // PathTracker.cpp:529-531. Without this, a 359.5° vs 0.0° edge would
            // diverge from the sim (which wraps via normalize_turn).
            float delta = next.heading_deg - inbound.heading_deg;
            while (delta >  180.0f) { delta -= 360.0f; }
            while (delta <= -180.0f) { delta += 360.0f; }
            if (fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG)
            {
                passThrough = true;
            }
        }
    }
}
_passThrough = passThrough;
```

Do NOT cache `nextNext` across ticks — it must be re-read each peek, since
`_destinationNode` may change between Mission steps. The wrap-normalize logic MUST
match the sim's `normalize_turn`; bit-for-bit equivalence is the AC8 contract.

**Optional refactor (T4):** extract the wrap-normalize loop into a `WROMath` free
helper (e.g. `wrapNormalize180(float deg)`) and reuse it in both PathTracker.cpp:529
and the new peek. Either choice is acceptable; the inline version above is the
minimal-change form.

```
LINE_FOLLOW:
  on Intersection.isAtIntersection():
    [run the peek logic above to set _passThrough]
    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(!_passThrough)
                .setOvershootMode(true)
                .setBrakeOnGate(!_passThrough)
                .setDistanceTarget(LINE_SENSOR_AXLE_OFFSET_CM)
    state = LINE_OVERSHOOT

LINE_OVERSHOOT:
  on LineFollower.isDistanceReached():
    Localization.onIntersection(_nextHop)
    state = PLAN_NEXT_HOP
    # Note: pass-through leaves LineFollower enabled; stop-and-go path already self-disabled.
    # _passThrough stays true through PLAN_NEXT_HOP and _dispatchEdgeAfterTurn.

_dispatchEdgeAfterTurn (LINE next):
  drain Intersection
  LineFollower.setEnabled(true).setPreset(preset).setOvershootMode(false)
  state = LINE_FOLLOW
  _passThrough = false  # consumed

_dispatchEdgeAfterTurn (PROJECTION next):
  drain Intersection
  LineFollower.setEnabled(true)
              .setPreset(preset)
              .setOvershootMode(false)
              .setDistanceTarget(edge.distance_cm)
  state = PROJECTION_FOLLOW
  _passThrough = false

_dispatchEdgeAfterTurn (BLIND next):
  if _passThrough:
    LineFollower.setEnabled(false).setBrake(false)   # release motors, no stop
    startSpeed = presetToPwm(current preset)
  else:
    startSpeed = Config::TSHAPE_START_SPEED  # today's default
  switch resolvedBlindMode:
    SYNC_TSHAPE: Drive.driveDist(d).setMode(SYNC).tShape()
                      .setStartSpeed(startSpeed).async()
    GYRO_TSHAPE: Drive.driveDist(d).setMode(GYRO).setHeading(imuTarget).tShape()
                      .setStartSpeed(startSpeed).async()
    RAW_FLAT:    Drive.driveDist(d).setMode(RAW)
                      .setStartSpeed(cruise).setEndSpeed(cruise).async()
                      # unchanged — already used cruise as start/end
  state = BLIND_FORWARD
  _passThrough = false
```

### Sim primitive emission

```python
# tools/path_planner/sim/runtime.py — module constants (mirror Config.h)

# Hand-mirrored from `Config::NAV_TURN_TOLERANCE_DEG` in WRO2026/Config.h.
# A pytest (TC9) verifies parity via regex grep — no C parser.
NAV_TURN_TOLERANCE_DEG: float = 0.5

# Hand-mirrored from `Config::PATH_PASS_THROUGH_ENABLED` in WRO2026/Config.h.
# A pytest (TC9 extended) verifies parity via the same regex pattern.
PATH_PASS_THROUGH_ENABLED: bool = True

# tools/path_planner/sim/runtime.py — go_to() / step_through()

if edge_type == EDGE_LINE:
    out.append((ACT_FOLLOW_INT, None))
    chassis.follow_line_to_intersection(distance_cm)
    # Peek next hop to decide pass-through vs overshoot.
    pass_through = False
    if PATH_PASS_THROUGH_ENABLED and nxt != goal_name:
        next_next = next_hop[nxt][goal_name]
        if next_next is not None and next_next != nxt:
            next_edge = graph.edges[nxt, next_next]
            # MUST use normalize_turn here — C++ side wrap-normalizes too (AC8 contract).
            heading_delta = normalize_turn(
                float(next_edge["heading_deg"]) - float(edge["heading_deg"])
            )
            if abs(heading_delta) <= NAV_TURN_TOLERANCE_DEG:
                pass_through = True
    if pass_through:
        out.append((ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM))
    else:
        out.append((ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM))
    chassis.forward(LINE_SENSOR_AXLE_OFFSET_CM)
    localization.on_intersection(nxt)
```

Both `NAV_TURN_TOLERANCE_DEG` and `PATH_PASS_THROUGH_ENABLED` follow the same hand-mirror
pattern as the existing `LINE_SENSOR_AXLE_OFFSET_CM` mirror at `runtime.py:80`. The parity
pytest (TC9) extends to cover both constants via regex grep on `Config.h`.

## Error Handling

- **Pass-through into a NEXT_HOP_NONE peek:** `FieldMap::getNextHop(B, dest)` returns
  `NODE_NONE`. `_passThrough = false`. Falls back to today's stop-and-go. No fault.
- **Pass-through with EdgeType::NONE peeked:** same — `_passThrough = false`, stop-and-go.
- **Pass-through where peeked heading puts the chassis past tolerance:** `_passThrough =
  false`, stop-and-go.
- **Line offline during 5 cm overshoot:** LineFollower dead-reckons symmetric cruise (per
  `setOvershootMode(true)`). The encoder gate still ticks; gate trips at 5 cm. No
  LOST_LINE_ACTION dispatch.
- **Chassis fault during overshoot:** `Drive.hasFault()` is polled at the top of every
  `update()` tick; tripping it enters `Fault::DRIVE_FAULT` and aborts, same as today.
- **Bad next-hop graph state (peek returns own node):** treat as `NODE_NONE` for safety. No
  fault.
- **Config::PATH_PASS_THROUGH_ENABLED = false:** every intersection is stop-and-go regardless
  of geometry. Useful for debug runs and as the rollback knob if the feature regresses on
  the chassis.

## Testing Strategy

**Levels:** Unit (sim primitives), Integration (sim end-to-end on fixture graphs), Hardware
(stopwatch + telemetry on real chassis, Phase 3 deliverable).

| ID   | Test Case                                                  | Type        | Expected Behavior                                                                                              |
|------|------------------------------------------------------------|-------------|----------------------------------------------------------------------------------------------------------------|
| TC1  | Sim: 4-node collinear LINE chain A→B→C→D goal              | Integration  | Primitive sequence: FOLLOW_INT, LINE_PASS, FOLLOW_INT, LINE_PASS, FOLLOW_INT, LINE_OVERSHOOT. Final node stops.    |
| TC2  | Sim: 90° turn at B (A→B→C with B→C orthogonal to A→B)      | Integration  | TURN_REL, FOLLOW_INT, LINE_OVERSHOOT, TURN_REL, FOLLOW_INT, LINE_OVERSHOOT. No LINE_PASS.                          |
| TC3  | Sim: heading delta just inside tolerance (0.4999°)         | Unit         | LINE_PASS fires (inclusive ≤). Avoid testing exact 0.5° equality — floating-point unstable.                        |
| TC4  | Sim: heading delta just outside tolerance (0.5001°)        | Unit         | LINE_OVERSHOOT (not LINE_PASS); subsequent turn fires.                                                             |
| TC5  | Sim: collinear LINE→PROJECTION at B                        | Integration  | FOLLOW_INT, LINE_PASS, FOLLOW_DIST. No turn between.                                                               |
| TC6  | Sim: collinear LINE→BLIND at B                             | Integration  | FOLLOW_INT, LINE_PASS, FORWARD. No turn between.                                                                   |
| TC7  | Sim: collinear chain where next-hop terminates at goal     | Unit         | The intersection where _nextHop == _destinationNode emits LINE_OVERSHOOT (never LINE_PASS).                        |
| TC8  | Sim: PATH_PASS_THROUGH_ENABLED mirror disables pass-through| Unit         | With Python mirror set to False, every LINE edge emits LINE_OVERSHOOT regardless of heading. Mirror lives in runtime.py alongside the existing `LINE_SENSOR_AXLE_OFFSET_CM` mirror. |
| TC9  | Sim: NAV_TURN_TOLERANCE_DEG mirror parity                  | Unit         | The mirrored constant in runtime.py matches `Config.h` via regex grep (`constexpr float NAV_TURN_TOLERANCE_DEG = (\d+\.\d+f)`). No C parser dependency. |
| TC10 | Sim: heading delta wrap (359.5° vs 0.0°)                   | Unit         | Normalized via `normalize_turn` to -0.5°; LINE_PASS fires. **Regression test for sim/C++ parity** — the C++ peek MUST also wrap-normalize (C8 of critic review). |
| TC11 | Sim: ACT_LINE_PASS chassis position update == OVERSHOOT    | Unit         | After LINE_PASS, chassis x/y advances by LINE_SENSOR_AXLE_OFFSET_CM at the inbound heading. Same as OVERSHOOT. Verifies positional invariant (tautology by construction, but guards against accidental drift if the chassis advance code is refactored).                          |
| TC12 | Sim: PROJECTION-inbound exit unchanged                      | Unit         | Path whose intersection node is reached via a PROJECTION edge (not LINE) emits the legacy sequence — no LINE_PASS. AC11 coverage.                                |
| T10a | Host-compile: setOvershootMode dead-reckon branch          | Unit (C++)   | Stub LineArray::isOnline() reads to synthesize "online → offline" sequence. With overshoot mode ON, controller dead-reckons symmetric cruise PWM (no LOST_LINE_ACTION dispatch). With overshoot mode OFF, today's behavior is preserved. |
| T10b | Host-compile: setBrakeOnGate(false) gate-trip behavior     | Unit (C++)   | Stub encoder reads to synthesize a 5 cm distance gate. With brake-on-gate=false, gate trip latches `isDistanceReached=true`, clears `_distanceTargetActive`, but does NOT disable the controller (no `Drive.stop`).                  |
| T10c | Host-compile: PATH_PASS_THROUGH_ENABLED behavioral toggle  | Unit (C++)   | Two build targets `-DPATH_PASS_THROUGH_ENABLED=1` and `=0`. Each asserts the correct primitive sequence on the canonical collinear fixture. Compile-only is not sufficient.                                                              |
| T10d | Host-compile: junction-freeze interaction in overshoot     | Unit (C++)   | Stub junction-classifier returns T_OR_CROSS during the overshoot window; assert `_pidFrozen` stays true and symmetric cruise PWM is still commanded. AC9 coverage.                                                                       |
| HW1  | Hardware: 4-node collinear LINE chain, stopwatch           | Hardware     | **Phase 3 deliverable** — chassis traverses A→D without visible deceleration at B, C.                              |
| HW2  | Hardware: turn-at-intersection regression                  | Hardware     | **Phase 3 deliverable** — chassis stop-turns at non-collinear intersections, identical to today.                   |
| HW3  | Hardware: line ends at node during overshoot               | Hardware     | **Phase 3 deliverable** — chassis advances the full 5 cm, no premature stop. Confirms AC6 latent-bug fix on real hardware (host-compile T10a covers the branch in isolation; HW3 covers debounce timing + sensor noise). |

**Test data:** Add a new collinear 4-node fixture file at
`tools/path_planner/tests/fixtures/collinear_4node.py` following the pattern of existing
fixtures in that directory (e.g. `toy_field_data.py`). The fixture exposes `NODES`, `EDGES`,
and a precomputed `NEXT_HOP` for a straight A→B→C→D run (all edges collinear within
`NAV_TURN_TOLERANCE_DEG`). Import it directly in the new test modules; do **not** introduce
a `conftest.py` — the repo's existing convention is per-fixture modules under
`tests/fixtures/`. TC5/TC6/TC12 extend the existing PROJECTION + BLIND fixtures.

**Host-compile harness:** The C++ host-compile tests (T10a–T10d) require a new
build-and-run harness under `WRO2026/test/`, modeled on the existing `arduino-cli compile`
CI invocation but targeting host gcc/clang with the Arduino stubs replaced by mocks (the
`MatrixMiniR4` API surface is small enough to mock the methods this PR touches:
`MiniR4.M2.getEncoder()`, `MiniR4.M3.getEncoder()`, `Drive.setSpeed`, `Drive.stop`,
`Drive.getEncL/R`, `LineArray.getError`, `LineArray.isOnline`, `LineArray.getJunctionType`).
**Setting up this harness is part of T10.** If the harness proves too costly to stand up in
this PR, the team may downgrade T10 to a hardware-only verification (HW3) and document the
test-coverage gap explicitly in ADR-023 — but the default plan is to build the harness.

**Run command:**
```
uv run pytest tools/path_planner/tests/
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/
# Host-compile tests (T10) — new harness under WRO2026/test/.
# Two build variants for AC7b:
make -C WRO2026/test PATH_PASS_THROUGH_ENABLED=1 test
make -C WRO2026/test PATH_PASS_THROUGH_ENABLED=0 test
```

The exact host-compile build system (CMake, Makefile, or a `tools/cppcheck/`-style script)
is an implementation detail of T10; whichever the implementer picks, both variants MUST run
in CI alongside the existing Arduino compile check.

## Tasks

| ID  | Task                                                         | Blocked By   | Risk | Files                                                                                            | Description |
|-----|--------------------------------------------------------------|--------------|------|--------------------------------------------------------------------------------------------------|-------------|
| T1  | Add Config::PATH_PASS_THROUGH_ENABLED constant + Doxygen     | —            | low  | `WRO2026/Config.h`                                                                               | Add `constexpr bool PATH_PASS_THROUGH_ENABLED = true;` in the navigation section near `NAV_TURN_TOLERANCE_DEG`. Full Doxygen (`@brief`, `@note`) explaining the rule, what it controls, and the rollback semantics. Satisfies AC7. |
| T2  | LineFollower header: setOvershootMode + setBrakeOnGate API   | —            | med  | `WRO2026/LineFollower.h`                                                                         | Add the two builder methods to `LineFollowerEnableBuilder` with full Doxygen (`@brief`, `@param`, `@return`, `@note`). Add the two private fields (`_overshootMode`, `_brakeOnGate`) to `LineFollowerClass` (plus `_hasOvershootModeOverride` / `_hasBrakeOnGateOverride` if needed — see Notes for Implementer). Add the matching `_applyEnableBuilder` signature parameters. Update `_applyEnableBuilder` callers' inline comments. Pure header changes; no behavior change yet. |
| T3  | LineFollower impl: overshoot-mode dead-reckon + brake-on-gate | T2           | med  | `WRO2026/LineFollower.cpp`                                                                       | Implement `setOvershootMode` + `setBrakeOnGate` on the builder. In `update()`, when `_overshootMode && !_array->isOnline()`, replace the LOST_LINE_ACTION dispatch with a symmetric `Drive.setSpeed(deadbandSnap(cruise), deadbandSnap(cruise))` and skip P+D, but **still tick the distance-gate evaluation** so the encoder gate fires after 5 cm. Reset `_lastErr = 0` on entry to the dead-reckon branch. Implement the gate-trip path: when `_brakeOnGate == false`, set `_distanceReached = true`, clear `_distanceTargetActive` + `_useTShape*`, but do NOT call `_applyEnableBuilder` and do NOT call `Drive.stop`. Default `_brakeOnGate = true` preserves today. Satisfies AC6 (combined with T10a). |
| T4  | PathTracker LINE_FOLLOW: peek + decide _passThrough           | T2           | high | `WRO2026/PathTracker.h`, `WRO2026/PathTracker.cpp`                                               | Add `bool _passThrough = false;` to `PathTrackerClass` private members. Modify the `Intersection.isAtIntersection()` branch in `update()`'s `LINE_FOLLOW` case using the exact peek code block in "Peek data sources" above. Chain `.setOvershootMode(true).setBrakeOnGate(!_passThrough).setTShapeRampDown(!_passThrough)` onto the existing `setEnabled(true)` builder. **Wrap-normalize the heading delta** (matches sim's `normalize_turn`) — without wrap, the 359.5°/0.0° boundary diverges from sim (AC8 / TC10 contract). Satisfies AC10. |
| T5  | PathTracker LINE_OVERSHOOT: branch on _passThrough            | T3, T4       | high | `WRO2026/PathTracker.cpp`                                                                        | In `update()`'s `LINE_OVERSHOOT` case, after `isDistanceReached()`: call `Localization.onIntersection(_nextHop)`, transition to `PLAN_NEXT_HOP`. Remove the unconditional `LineFollower.setEnabled(false)` — let it be governed by `_brakeOnGate` (already disabled internally for stop-and-go; left enabled for pass-through). Add a docstring noting that `_passThrough` is consumed in the upcoming dispatch (T6/T7). Satisfies AC5. |
| T6  | PathTracker dispatch: LINE/PROJECTION handoff                 | T5           | high | `WRO2026/PathTracker.cpp`                                                                        | In `_dispatchEdgeAfterTurn`, LINE branch: add `.setOvershootMode(false)` to the existing `setEnabled(true).setPreset(preset)` chain. PROJECTION branch: add `.setOvershootMode(false)`. After the dispatch, set `_passThrough = false`. Justified by the apply-builder fast-path (already-enabled → no ramp-up reseed). Satisfies AC4. |
| T7  | PathTracker dispatch: BLIND handoff with startSpeed           | T6           | high | `WRO2026/PathTracker.cpp`                                                                        | In `_dispatchEdgeAfterTurn`, BLIND branch: when `_passThrough` is set, first capture `cruisePwm = presetToPwm(LineFollower.getPreset())` BEFORE disabling, then disable LineFollower with `.setBrake(false)`, then chain `.setStartSpeed(cruisePwm)` onto SYNC_TSHAPE and GYRO_TSHAPE Drive builders. RAW_FLAT already uses cruise as start/end so no change. Reset `_passThrough = false`. Satisfies AC3. |
| T8  | Sim: ACT_LINE_PASS primitive vocab + go_to/step_through       | —            | med  | `tools/path_planner/sim/runtime.py`                                                              | Add `ACT_LINE_PASS = "line_pass"` next to `ACT_LINE_OVERSHOOT`. In `go_to()` and `step_through()`: after `chassis.follow_line_to_intersection`, peek `next_hop[nxt][goal_name]`. If non-None, not the goal, AND heading delta (wrapped via `normalize_turn`) within `NAV_TURN_TOLERANCE_DEG`, emit `ACT_LINE_PASS` (replacing `ACT_LINE_OVERSHOOT`). Same 5 cm chassis advance. Hand-mirror `NAV_TURN_TOLERANCE_DEG` and `PATH_PASS_THROUGH_ENABLED` from Config.h as module constants. Satisfies AC8. |
| T9  | Sim tests: collinear / turn / boundary fixtures               | T8           | low  | `tools/path_planner/tests/test_sim.py`, `tools/path_planner/tests/fixtures/collinear_4node.py` (NEW) | Add a new fixture file `collinear_4node.py` (following existing `fixtures/*.py` convention — do NOT add a `conftest.py`). Write TC1-TC12 from the testing table. Use parametrized tests where the same setup applies (e.g., LINE_PASS vs LINE_OVERSHOOT branching). Verify mirrored `NAV_TURN_TOLERANCE_DEG` matches `Config.h` via regex grep (TC9). |
| T10 | Host-compile harness + C++ unit tests (T10a–T10d)             | T3           | high | `WRO2026/test/` (NEW directory), `WRO2026/test/test_line_follower_overshoot.cpp` (NEW), CI config | Stand up a host-compile test harness under `WRO2026/test/`. Mock the MatrixMiniR4 surface used by LineFollower/PathTracker (small footprint — `Drive.setSpeed/stop/getEncL/R`, `LineArray.getError/isOnline/getJunctionType`, `MiniR4.M2/M3.getEncoder`). Implement test cases T10a (overshoot-mode dead-reckon), T10b (brake-on-gate=false), T10c (PATH_PASS_THROUGH_ENABLED toggle on both build variants), T10d (junction-freeze + overshoot interaction). Add both build variants to the existing Arduino CI workflow. If the harness proves too costly to land in this PR, downgrade to HW3-only and document the gap explicitly in ADR-023 — but the default plan is to build the harness. Satisfies AC6, AC7b, AC9. |
| T11 | ADR-023: pass-through + overshoot-mode runtime probe         | T4, T7       | low  | `docs/decisions/023-pass-through-collinear-intersections.md` (NEW), `DECISIONS.md`               | Write ADR per the project template (Status / Date decided / Rule / Why / How to apply / Trigger to revisit / See also). **ADR-023 explicitly owns the overshoot-mode contract** — not "drive-by improvement" framing. Rule clause includes the runtime line-presence probe semantics (per critic C4). Cross-link: "Refines ADR-018 (distance-gated stops)" — ADR-018 stays valid as the default; ADR-023 defines the runtime-probe override. Cite ADR-020 (junction PID freeze) under See also. Update DECISIONS.md index + status-summary table; mark ADR-018 as "Refined by ADR-023". |
| T12 | EventTrace + telemetry sanity                                 | T3           | low  | `WRO2026/EventTrace.cpp` (READ-ONLY check), `WRO2026/LineFollower.cpp` (optional comment)        | Verify no EventTrace code change is needed — `LineFollower::isLineLost()` accessor must still reflect the underlying line state even when `setOvershootMode(true)` suppresses LOST_LINE_ACTION. Add a one-line comment in LineFollower.cpp clarifying this if not already obvious. |

**Sequential ordering (must be respected):**
- **T5 → T6 → T7 must run sequentially in this order.** All three edit overlapping regions
  of `WRO2026/PathTracker.cpp` (`_dispatchEdgeAfterTurn`, `update()`, the `_passThrough`
  member). Do not dispatch them as parallel sub-agents — they will conflict.
- T2 and T8 are root tasks (no blockers) and can run in parallel.
- T3 follows T2.
- T4 follows T2 (needs the new builder API).
- T10 follows T3 (needs the new LineFollower behavior to test).
- T9 follows T8.
- T11 (ADR) follows the substantive C++ changes (T4 + T7) — once API contract is fixed.

## Notes for Implementer

- **Junction-freeze interaction:** when both junction-freeze (`_pidFrozen`) and
  overshoot-mode are active simultaneously (chassis is past the marker, classifier still
  reads non-NONE — possible briefly), both branches emit symmetric cruise. Result is
  consistent. No special handling needed.
- **`_useTShapeRampDown` reset semantics:** today, the gate trip path in `update()` calls
  `_applyEnableBuilder(targetEnabled=false, ...)` which clears `_useTShapeRampDown`. With
  `_brakeOnGate=false`, the gate-trip code must explicitly clear `_useTShapeRampDown`
  (and `_useTShape`, `_distanceTargetActive`) without going through `_applyEnableBuilder`.
- **`_overshootMode` lifecycle (decided — match `_hasPresetOverride` pattern):**
  `setOvershootMode(bool)` sets `_hasOvershootModeOverride = true` and stores the value in
  `_overshootModeOverride`. `_applyEnableBuilder` honors the override only when the flag is
  set; without an explicit `setOvershootMode(...)` call, the previous `_overshootMode` value
  is preserved across re-enables (same semantics as `_preset` and `_reverse` today). The
  PathTracker code in T6 must therefore chain `.setOvershootMode(false)` on the next LINE /
  PROJECTION re-arm to flip back to normal P+D — already specified in T6's description.
- **Drive cruise capture for BLIND handoff (T7) — CORRECTED:**
  `LineFollower::_applyEnableBuilder`'s disable branch (`LineFollower.cpp:557-577`) does
  **NOT** clear `_preset` — `_preset` is intentionally preserved across enable/disable
  cycles (see `setEnabled` doxygen). Capture `cruisePwm = presetToPwm(LineFollower.getPreset())`
  in PathTracker BEFORE the disable call out of defensive pattern (in case future changes
  to `_applyEnableBuilder` touch `_preset`), but the current LineFollower behavior makes
  the order-of-operations safe either way. (Previous draft of this note incorrectly
  claimed `_preset` was cleared on disable — fixed per critic C9.)
- **Encoder baseline race:** at intersection-detect in LINE_FOLLOW, the new builder chain
  includes `.setDistanceTarget(5)`. The `_applyEnableBuilder` path resets the encoder
  baseline (`_distanceStartEncL/R = Drive.getEncL/R()`). This happens atomically inside the
  builder destructor; no race between distance gate and ramp-down ramp baseline.
- **Sim mirror parity (T9):** the `NAV_TURN_TOLERANCE_DEG` mirror test must use a regex
  grep against `Config.h`, not a C parser (the project intentionally avoids that
  dependency — see the existing `LINE_SENSOR_AXLE_OFFSET_CM` mirror in runtime.py:80).
- **Sim chassis position update:** `chassis.forward(LINE_SENSOR_AXLE_OFFSET_CM)` is
  identical for `ACT_LINE_PASS` and `ACT_LINE_OVERSHOOT`. The primitive name differs only
  for downstream verification.
- **Rollback plan (two-level):**
  - **Level 1 (pass-through only):** if pass-through regresses on hardware, set
    `Config::PATH_PASS_THROUGH_ENABLED = false`. The overshoot-mode change to LineFollower
    stays in effect (still dead-reckons during overshoot), but every intersection becomes
    stop-and-go. AC7b confirms this path compiles and behaves correctly.
  - **Level 2 (overshoot-mode rollback):** if the ADR-023 overshoot-mode runtime probe
    itself regresses, the rollback is a code change — chain `.setOvershootMode(false)` on
    the LINE_FOLLOW arming in PathTracker (T4) — which restores today's LOST_LINE_ACTION
    dispatch during overshoot. This is a project-wide rollback of the ADR-018 refinement
    and would require a follow-up ADR amendment.
