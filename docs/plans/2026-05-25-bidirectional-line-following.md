# Bidirectional Line Following (Issue #21) Plan

**Goal:** Let the chassis traverse line edges using either the front-board (forward) or rear-board (reverse) sensor — auto-picked at runtime when reverse saves turn-degrees, and overrideable from three layers (per-edge in `field_data.py`, per-call `Mission.goTo(...).setDirection(...)`, per-call `PathTracker.goTo(...)` builder).

**Approach:** Auto direction lives at runtime in `PathTracker._onPlanNextHop` (debate-decided — planner-side requires line-graph state expansion the current `NEXT_HOP[u][v]` schema cannot represent). At every plan step the FSM compares `|wrap(targetHeading − currentHeading)|` against a `BIDIRECTIONAL_TURN_THRESHOLD_DEG` (default 90°): if reverse is strictly cheaper *and* `LINE_BOARD_COUNT == 2` *and* `BIDIRECTIONAL_ENABLED`, dispatch with `LineFollower.setReverse(true)` and `Drive` negative-power for BLIND/PROJECTION. Manual overrides resolve in strict layered precedence: per-edge `direction_override` > builder `setArrivalDirection` (last hop only) > builder `setRouteDirection` (all hops) > auto. Sim mirrors the algorithm primitive-for-primitive with three new tags (`ACT_LINE_REVERSE`, `ACT_BLIND_REVERSE`, `ACT_PROJECTION_REVERSE`).

## Scope

- **In scope:**
  - Auto front/rear direction selection at edge dispatch, gated by `BIDIRECTIONAL_ENABLED` + `LINE_BOARD_COUNT == 2`.
  - `Direction` enum (`FRONT`, `REAR`, `AUTO`) on `PathTracker::goTo` builder via two modifiers: `.setRouteDirection(Direction)` (every hop) and `.setArrivalDirection(Direction)` (final hop only).
  - Mission DSL modifier verbs `setRouteDirection(Direction)` / `setArrivalDirection(Direction)` that mutate the most recently enqueued `GOTO` step (no-op + warn otherwise).
  - Per-directed-edge `direction` key (`"front"` | `"rear"`) in `field_data.py` EDGES, validated, asymmetric (only applies to `(u,v)`, not `(v,u)`), emitted as `EdgeInfo::direction_override` ∈ `{DEFAULT, FORCE_FORWARD, FORCE_REVERSE}` in generated `routes.h`.
  - Auto-direction logic applies to ALL edge types (LINE, BLIND, PROJECTION). BLIND/PROJECTION reverse is implemented via negative `Drive` power; LINE/PROJECTION reverse also flips `WROLineArray._activeBoard` to the rear board via the existing `LineFollower::setReverse(bool)` API.
  - Master kill-switch `Config::BIDIRECTIONAL_ENABLED` (default `true`).
  - Sim mirroring: three new primitives + extended `PathTrackerSim.go_to`. Existing `feedback_sim_mirrors_robot` rule preserved.
  - Tests: parity grep for new Config constants (TC9-style), Codex P1-style regression that `direction` is not mirrored, sim test cases for forward/reverse auto-selection, pass-through interaction tests, host-compile test for `_onPlanNextHop`.
  - ADR-024 documenting the precedence rule, the 90° threshold, and the layer choice (runtime vs planner).

- **Out of scope:**
  - **Tasks layer integration.** Tasks doesn't exist yet (Roadmap P5/P6). The Mission DSL modifiers are designed so Tasks can adopt the same pattern, but no Task-side code lands in #21.
  - **Junction `LEFT`/`RIGHT` semantics under reverse.** Today no module reads `JunctionType::LEFT`/`RIGHT` for routing logic — only `T`/`X`/`CROSS` are consumed for node-advance. When a future issue adds LEFT/RIGHT-aware code, it will handle reverse-mode semantics then. (Note in ADR-024 that the rear board's ch0 is still chassis-left per `WROLineArray.h:113-119`, so weight signs do not change.)
  - **State-expanded Dijkstra / turn cost in the planner.** Decided against in the debate — planner stays direction-agnostic; both directed edges of every segment retain equal cost.
  - **1-board chassis "drive forward through reverse"** — i.e., negating motor power on a 1-board chassis to traverse a line edge backwards using the same front board. Mechanically possible but the front sensor would be trailing; line-follow stability is undefined. `LINE_BOARD_COUNT == 1` + REAR on a LINE edge raises `Fault::REVERSE_UNAVAILABLE`. BLIND/PROJECTION reverse still works on 1-board configs (no sensor involved).
  - **Per-Mission-step direction logging in OLED/EventTrace.** Eventually visible via `LineFollower.getReverse()` state probe but no dedicated event tag in #21.

## Decisions

- **Auto-decision lives at runtime, in `PathTracker._onPlanNextHop`** — debate verdict 3.0/10 against planner-side; planner-side requires line-graph state expansion the current NEXT_HOP `[u][v]` schema cannot represent.
- **Threshold = 90° strict, ties go forward** — `|delta| > BIDIRECTIONAL_TURN_THRESHOLD_DEG` (default 90.0f) triggers reverse; `==` ties to forward. Geometric constant, not a tuning parameter.
- **Precedence: per-edge > arrival-builder > route-builder > auto** — strict layering. Edge override is binding (mechanical constraint); arrival is more specific than route (last-hop vs all-hops); auto only fires when all three are `DEFAULT`/`AUTO`.
- **Mission DSL uses modifier verbs that mutate the last enqueued step** — `Mission.goTo("A").setRouteDirection(Direction::REAR).setArrivalDirection(Direction::FRONT)`. Chain-style consistent with existing `Mission` verb-chain (`MissionClass&` return), no builder-return refactor.
- **Per-edge key is `"direction": "front"` | `"rear"`** — asymmetric per directed edge (matches existing `_DIRECTIONAL_OPTIONAL_KEYS` pattern from Codex P1 regression).
- **Sim primitives parallel to forward variants** — `ACT_LINE_REVERSE`, `ACT_BLIND_REVERSE`, `ACT_PROJECTION_REVERSE`, mirroring how `ACT_LINE_PASS` shipped in #19.
- **`BIDIRECTIONAL_ENABLED` defaults to `true`** — mirror `PATH_PASS_THROUGH_ENABLED_DEFAULT` (ADR-023). Bidirectional becomes the new normal whenever 2-board is configured.

## Acceptance Criteria

- [ ] **AC1** With `LINE_BOARD_COUNT == 2` and `BIDIRECTIONAL_ENABLED == true`, if the chassis is at world heading 0° and the next edge has heading 175°, the FSM dispatches the edge with `LineFollower.setReverse(true)` instead of turning 175° (turning 5° to align rear-board with edge); chassis advances along the edge using the rear board.
- [ ] **AC2** With `LINE_BOARD_COUNT == 2` and `BIDIRECTIONAL_ENABLED == true`, if the chassis is at heading 0° and the next edge has heading 45°, the FSM turns 45° and dispatches forward (`setReverse(false)`). Reverse is not used; `|45°| <= 90°` threshold.
- [ ] **AC3** Exact tie: heading delta = 90°. FSM picks forward (`setReverse(false)`). Threshold is strict `>`, ties go forward.
- [ ] **AC4** `Mission.goTo("A").setArrivalDirection(Direction::REAR)` causes only the FINAL hop into A to be dispatched reverse; intermediate hops use auto. Verified by sim with a 3-node path where the chassis-natural direction would be forward for intermediate hops.
- [ ] **AC5** `Mission.goTo("A").setRouteDirection(Direction::REAR)` causes EVERY hop on the path to A to be dispatched reverse, regardless of geometry. If a hop has `direction_override == FORCE_FORWARD` in `routes.h`, the edge override wins (precedence).
- [ ] **AC6** Both modifiers can be combined: `Mission.goTo("A").setRouteDirection(Direction::FRONT).setArrivalDirection(Direction::REAR)` — all hops forward except the last, which is reverse.
- [ ] **AC7** Per-directed-edge override: `EDGES = [{"u":"A", "v":"B", "type":"LINE", "direction":"rear"}, ...]` in `field_data.py` emits `direction_override = FORCE_REVERSE` on `EdgeInfo::EDGES[A][B]` ONLY. `EDGES[B][A].direction_override == DEFAULT` (asymmetric; mirrors `pre_turn_mode` and Codex P1 invariant).
- [ ] **AC8** Edge override beats all builder overrides. If `EDGES[A][B].direction_override == FORCE_FORWARD` and Mission says `setArrivalDirection(REAR)`, edge wins — chassis approaches A forward on the last hop.
- [ ] **AC9** With `BIDIRECTIONAL_ENABLED == false`, auto never picks reverse. Builder/edge overrides still work (operator can force reverse manually).
- [ ] **AC10** With `LINE_BOARD_COUNT == 1` and a LINE edge resolved to REAR (by any means), `PathTracker` enters `Fault::REVERSE_UNAVAILABLE` with `cur` and `nxt` populated. Auto never resolves to REAR on a 1-board configuration.
- [ ] **AC11** BLIND edge resolved to REAR dispatches `Drive.forwardGyro(-power)` or `Drive.driveDist(...).setPower(-cruise)` (negative). Chassis moves backward along the edge. No board switching (no sensor involved).
- [ ] **AC12** PROJECTION edge resolved to REAR dispatches `LineFollower.setEnabled(true).setReverse(true)` + distance gate. Rear board reads the line; chassis traverses backward to the projected stop point.
- [ ] **AC13** Pass-through (ADR-023) composes with reverse via *body-heading* delta (not raw forward edge headings): when two consecutive hops have resolved body-headings that differ by ≤ `NAV_TURN_TOLERANCE_DEG` (regardless of whether each hop is FRONT or REAR), pass-through fires across the intersection without overshoot or turn. The peek must use resolved body-headings, not raw edge headings — required for mixed REAR↔FORWARD transitions on anti-parallel forward edges.
- [ ] **AC14** `tools/path_planner/sim/runtime.py::PathTrackerSim.go_to` emits `ACT_LINE_REVERSE` (and `ACT_BLIND_REVERSE` / `ACT_PROJECTION_REVERSE`) for every hop the runtime resolves to REAR. `compile_path` produces the same primitive sequence as the sim.
- [ ] **AC15** Config parity: a TC9-style regex test confirms `Config::BIDIRECTIONAL_ENABLED` and `Config::BIDIRECTIONAL_TURN_THRESHOLD_DEG` are defined in `Config.h` (sim hand-mirrors them as Python module constants).

## Data Model

### New / changed C++ enums

```cpp
// NavTypes.h — NEW standalone header to host the Direction enum. Avoids a
// circular include if Mission.h needs to reference Direction without
// pulling in all of PathTracker.h (which transitively includes FieldMap +
// routes.h). Both PathTracker.h and Mission.h include NavTypes.h.
enum class Direction : uint8_t
{
    AUTO  = 0,    // defer to per-edge override → auto algorithm
    FRONT = 1,    // force forward (chassis front leads)
    REAR  = 2,    // force reverse (chassis rear leads; LINE/PROJECTION use rear board)
};
```

**Enum interchange note:** `Direction::AUTO` (NavTypes.h) and `Routes::DirectionMode::DEFAULT` (routes.h) both encode "no override" but are distinct C++ types. Never compare cross-type — the resolution algorithm tests each independently (`if (builder == Direction::AUTO)` vs `if (edge_override == DirectionMode::DEFAULT)`). This mirrors how `Config::SpeedPreset` and `Routes::LineMode` already coexist.

```cpp
// routes.h — auto-generated, used on EdgeInfo.
enum class DirectionMode : uint8_t
{
    DEFAULT       = 0,    // defer to PipelineConfig.default_direction_mode → builder → auto
    FORCE_FORWARD = 1,    // edge always traversed front-leading
    FORCE_REVERSE = 2,    // edge always traversed rear-leading
};
```

### Changed structs

```cpp
// routes.h — EdgeInfo gains direction_override (placed at the end to keep
// existing field offsets stable for any host-side tooling that already
// parses the header).
struct EdgeInfo
{
    EdgeType       type;
    float          distance_cm;
    float          heading_deg;
    LineMode       line_mode_override;
    BlindMode      blind_mode_override;
    ProjectionMode projection_mode_override;
    TurnMode       pre_turn_mode_override;
    DirectionMode  direction_override;       // NEW
};

// routes.h — PipelineConfig gains default_direction_mode (DEFAULT means AUTO
// at runtime; FORCE_FORWARD/FORCE_REVERSE means project-wide override).
struct PipelineConfig
{
    LineMode       default_line_mode;
    BlindMode      default_blind_mode;
    ProjectionMode default_projection_mode;
    TurnMode       default_turn_mode;
    DirectionMode  default_direction_mode;   // NEW
};
```

```cpp
// Mission.h — Step grows direction state on GOTO steps (zero-init means
// AUTO on both fields, so existing missions are unchanged).
struct Step
{
    StepType    type;
    uint32_t    duration_ms;
    uint16_t    freq_hz;
    uint8_t     goal;
    uint8_t     r; uint8_t g; uint8_t b;
    const char* text;
    Direction   route_direction;       // NEW — defaults to Direction::AUTO
    Direction   arrival_direction;     // NEW — defaults to Direction::AUTO
};
```

```cpp
// PathTracker.h — NavEntry (FIFO slot consumed by _onPlanNextHop) grows
// the same two Direction fields so PathTracker.goTo builder can stage
// them per-hop. Zero-init means AUTO; existing call sites that don't
// touch these fields behave as before.
struct NavEntry
{
    uint8_t   nodeId;
    Direction route_direction;         // NEW — defaults to Direction::AUTO
    Direction arrival_direction;       // NEW — defaults to Direction::AUTO
};
```

### New Config constants

```cpp
// Config.h
#ifndef BIDIRECTIONAL_ENABLED_DEFAULT
#define BIDIRECTIONAL_ENABLED_DEFAULT true
#endif
constexpr bool  BIDIRECTIONAL_ENABLED            = BIDIRECTIONAL_ENABLED_DEFAULT;
constexpr float BIDIRECTIONAL_TURN_THRESHOLD_DEG = 90.0f;
```

### Python schema additions

`field_data.py` EDGES dict entry gains optional `"direction"` key:

```python
EDGES = [
    {"type": "LINE", "u": "A", "v": "B", "direction": "rear"},   # NEW key
    {"type": "LINE", "u": "C", "v": "D", "direction": "front"},
    # ... existing fields (distance_cm, heading_deg, primitive_mode, pre_turn_mode, pivot) unchanged
]
```

Validator schema: `direction` must be `"front"`, `"rear"`, or omitted (treated as `DEFAULT`). Asymmetric — only forwards onto `(u → v)`, not the reverse edge (added to `_DIRECTIONAL_OPTIONAL_KEYS` in `graph_builder.py`).

`pipeline_config.py` gains `default_direction_mode: DirectionMode` (default `DEFAULT` which means AUTO at runtime).

## API Contracts

### `PathTracker::goTo` builder

```cpp
PathTrackerGoToBuilder PathTracker.goTo(uint8_t goalId);
PathTrackerGoToBuilder PathTracker.goTo(const char* goalName);

class PathTrackerGoToBuilder
{
public:
    PathTrackerGoToBuilder& async();
    PathTrackerGoToBuilder& setRouteDirection(Direction d);     // NEW — applies to every hop
    PathTrackerGoToBuilder& setArrivalDirection(Direction d);   // NEW — applies to final hop only
    // destructor enqueues into PathTracker FIFO with the staged config
};
```

Defaults: `route_direction = AUTO`, `arrival_direction = AUTO`, `async = false`.

### `Mission` modifier verbs

```cpp
MissionClass& Mission.goTo(const char* nodeName);                // unchanged
MissionClass& Mission.setRouteDirection(Direction d);            // NEW — mutates last GOTO step
MissionClass& Mission.setArrivalDirection(Direction d);          // NEW — mutates last GOTO step
```

If the previous step is not `GOTO`, the modifier logs `[Mission] setRouteDirection: no preceding goTo step (ignored)` to Serial and returns `*this` unchanged. (Same defensive behavior as today's no-op on out-of-state verb chaining.)

### Resolution algorithm (canonical, mirrored in C++ and sim)

```python
# Pseudocode — same in PathTracker.cpp::_resolveDirection and sim/runtime.py.
def resolve_direction(
    edge_dir_override: DirectionMode,           # from EdgeInfo
    pipeline_default: DirectionMode,            # from PipelineConfig
    builder_arrival: Direction,                 # this hop is the last hop iff nxt == destination
    builder_route: Direction,                   # all hops
    current_heading: float,                     # IMU
    edge_heading: float,                        # from routes
    is_last_hop: bool,
    bidirectional_enabled: bool,
    line_board_count: int,
) -> Direction:
    # Step 1: per-edge override is binding.
    if edge_dir_override == FORCE_FORWARD: return FRONT
    if edge_dir_override == FORCE_REVERSE: return REAR

    # Step 2: pipeline default is binding when not DEFAULT.
    if pipeline_default == FORCE_FORWARD: return FRONT
    if pipeline_default == FORCE_REVERSE: return REAR

    # Step 3: arrival override wins on the final hop.
    if is_last_hop and builder_arrival != AUTO: return builder_arrival

    # Step 4: route override wins on all hops.
    if builder_route != AUTO: return builder_route

    # Step 5: auto — only if 2-board chassis AND kill-switch on.
    if not bidirectional_enabled: return FRONT
    if line_board_count < 2: return FRONT          # auto can't pick REAR without rear board

    delta = wrap_pm180(edge_heading - current_heading)
    if abs(delta) > BIDIRECTIONAL_TURN_THRESHOLD_DEG:
        return REAR
    return FRONT
```

`pipeline_default` is sourced from `Routes::PIPELINE_CONFIG.default_direction_mode` (the new field T3 adds). `edge_dir_override` is `EdgeInfo::direction_override` for the current `(cur → nxt)` edge.

### Sim primitives

New tuples in `tools/path_planner/sim/runtime.py`:

```python
ACT_LINE_REVERSE       = "line_reverse"          # (ACT_LINE_REVERSE, target_node, line_mode)
ACT_BLIND_REVERSE      = "blind_reverse"         # (ACT_BLIND_REVERSE, target_node, distance_cm, blind_mode)
ACT_PROJECTION_REVERSE = "projection_reverse"    # (ACT_PROJECTION_REVERSE, target_node, distance_cm, projection_mode)
```

`VirtualChassis.reverse(distance_cm: float)` mirrors `VirtualChassis.forward` but negates the displacement vector.

## Error Handling

- **`LINE_BOARD_COUNT == 1` + REAR resolved on a LINE/PROJECTION edge** — `PathTracker` enters `Fault::REVERSE_UNAVAILABLE` (new fault enum value), logs `[PathTracker] REVERSE_UNAVAILABLE: edge LINE/PROJECTION at u=<cur> v=<nxt> requires rear board`, halts via the existing fault-beep-flash. Auto never resolves to REAR on a 1-board chassis (Step 5 short-circuits), so this fault only fires when an operator or edge explicitly forces REAR.
- **Mission modifier on non-GOTO step** — `setRouteDirection`/`setArrivalDirection` log a warning to Serial and return `*this` unchanged. Mission state is not corrupted.
- **Invalid `direction` value in `field_data.py`** — validator raises `ValidationError("EDGES[i].direction must be 'front', 'rear', or absent; got <value>")` at build-routes time. CI catches it before flashing.
- **Heading wrap edge cases** — `delta` is normalized to `(-180°, +180°]` (same `while` loops as `PathTracker.cpp:587-589`); a 179.5° delta correctly resolves to REAR.

## Testing Strategy

**Levels:** Unit (Python validator/graph), Sim (Python `PathTrackerSim` end-to-end), Host-compile (C++ `_onPlanNextHop` against mocks), Integration (Arduino on hardware — out of CI but exercised before merge).

| ID    | Test Case                                                       | Type     | Expected Behavior                                                                                                                             |
|-------|-----------------------------------------------------------------|----------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| TC1   | Heading delta 91° → REAR; 89° → FRONT; 90° → FRONT              | Sim      | `resolve_direction` returns the expected enum at each boundary. Asserts in `tests/test_runtime_sim.py`.                                       |
| TC2   | Heading delta 179.5° (near antiparallel) → REAR                 | Sim      | Reverse rear-board sees a ~0.5° edge-aligned approach; FSM does not turn 179.5° to face forward.                                              |
| TC3   | `BIDIRECTIONAL_ENABLED == false` + delta 175° → FRONT (turn)    | Sim      | Auto disabled; chassis turns 175° instead of reversing.                                                                                       |
| TC3b  | AC9 clause (b): kill-switch off + explicit `setRouteDirection(REAR)` on a delta-20° segment → resolves REAR | Sim | Flag disables *auto* reversal only; a manual builder override must not be suppressed by the flag. Verifies clause (b) of AC9 independently from TC3. |
| TC4   | `LINE_BOARD_COUNT == 1` + delta 175° + LINE edge → FRONT (turn) | Sim      | Auto can't pick REAR without rear board; chassis turns.                                                                                       |
| TC5   | Mission `setArrivalDirection(REAR)` on 3-hop path               | Sim      | First two hops auto; last hop forced REAR. Compare emitted primitive sequence: `[ACT_FOLLOW_INT, ACT_FOLLOW_INT, ACT_LINE_REVERSE]`.          |
| TC6   | Mission `setRouteDirection(REAR)` on 3-hop path                 | Sim      | All three hops REAR: `[ACT_LINE_REVERSE, ACT_LINE_REVERSE, ACT_LINE_REVERSE]`.                                                                |
| TC7   | Per-edge `direction: "rear"` on (A,B), Mission says FRONT       | Sim      | Edge override wins; that hop is REAR despite operator preference.                                                                             |
| TC8   | Per-edge override is asymmetric                                 | Unit     | `EDGES = [{"u":"A","v":"B","direction":"rear"}]` produces `EDGES[A][B].direction_override == FORCE_REVERSE` but `EDGES[B][A] == DEFAULT`.     |
| TC9   | Pass-through with two consecutive REAR hops (collinear)         | Sim      | Body-headings collinear within `NAV_TURN_TOLERANCE_DEG` → `ACT_LINE_PASS` fires; no `LINE_OVERSHOOT` between them.                            |
| TC9b  | Pass-through does NOT fire when consecutive REAR hops are non-collinear | Sim | Two consecutive REAR hops whose forward headings differ by 90° (e.g., 0° then 90°, body-headings 180° and 270°, delta = 90° > NAV_TURN_TOLERANCE_DEG) → `ACT_LINE_PASS` does NOT fire; PathTracker emits a turn primitive. Negative test for AC13. |
| TC9c  | Pass-through fires on mixed REAR→FORWARD transition (anti-parallel edges) | Sim | Three-node fixture: hop A→B resolves REAR (forward heading 0°, body heading 180°), hop B→C resolves FRONT (forward heading 180°, body heading 180°). Body-heading delta = 0° → `ACT_LINE_PASS` fires at B. Old raw-edge-heading peek (delta=180°) would miss this — confirms peek uses resolved body-headings. |
| TC10  | BLIND edge REAR dispatches negative power                       | Sim      | `chassis.reverse(distance_cm)` invoked; chassis position advances along edge in the reverse direction.                                        |
| TC11  | PROJECTION edge REAR uses rear board + distance gate            | Sim      | `ACT_PROJECTION_REVERSE` primitive with the expected `distance_cm`.                                                                           |
| TC12  | `LINE_BOARD_COUNT == 1` + REAR on LINE → Fault                  | C++ host | Host-compile harness loads a fixture edge resolved REAR; asserts `PathTracker.getFault() == Fault::REVERSE_UNAVAILABLE` AND `getFaultInfo().fromNode == cur` AND `getFaultInfo().destNode == nxt` (fault trapped on the correct edge, not a stale latch). |
| TC13  | Mission modifier on non-GOTO step is a no-op                    | C++ host | `Mission.wait(100).setRouteDirection(REAR)` — wait step unmodified; Serial sees warning.                                                      |
| TC14a | TC9-style parity: `BIDIRECTIONAL_ENABLED` in Config.h           | Unit     | `tests/test_pass_through.py::test_tc14a_config_parity` greps `Config.h` for `#define\s+BIDIRECTIONAL_ENABLED_DEFAULT\s+true` and `constexpr\s+bool\s+BIDIRECTIONAL_ENABLED\s*=`. Anchored to defend against false positives. |
| TC14b | TC9-style parity: `BIDIRECTIONAL_TURN_THRESHOLD_DEG`            | Unit     | Anchored regex: `constexpr\s+float\s+BIDIRECTIONAL_TURN_THRESHOLD_DEG\s*=\s*90\.0f`. Bare `90.0f` grep is too loose — could match any matching literal in Config.h. |
| TC14c | Compile-time override branch coverage (`-DBIDIRECTIONAL_ENABLED_DEFAULT=false`) | C++ host | Build the host-compile harness twice — once with the default (`true`) and once with `-DBIDIRECTIONAL_ENABLED_DEFAULT=false`. On the same 175° fixture, the `true` build resolves REAR; the `false` build resolves FRONT. Confirms the `#ifndef` gate actually controls runtime behavior. |
| TC15  | Codex P1 regression: `direction` not mirrored                   | Unit     | `tests/test_planner.py` — EDGES override on (A,B) with `direction: "rear"` produces no override on (B,A).                                     |
| TC16  | Schema validator rejects invalid direction                      | Unit     | Validator raises `ValidationError` for `direction: "backwards"`.                                                                              |
| TC17  | Builder defaults are `AUTO`                                     | Unit/Sim | A bare `Mission.goTo("A")` (no modifier) produces `route_direction == AUTO, arrival_direction == AUTO`; auto algorithm runs.                  |
| TC18  | Compiler and sim agree on primitive sequence                    | Sim      | For a fixture path, `compile_path(...)` and `PathTrackerSim.go_to(...)` emit identical primitive sequences (existing parity invariant). Requires T12 (reverse-primitive emission in compiler) before any fixture with REAR hops can pass this test.                                                                                              |

**Test data:** Existing `tests/fixtures/toy_field_data.py` + new `tests/fixtures/bidirectional_field_data.py` (a path with one collinear-reverse chain and one mixed-direction route). Sim tests use mocked Config constants via `runtime.py` module-level overrides (as `LINE_SENSOR_AXLE_OFFSET_CM` already does).

**Run commands:**
- Python: `uv run pytest tools/path_planner/tests/`
- C++ host-compile: as in existing `WRO2026/test/` harness — see `WRO2026/test/test_path_tracker_passthrough.cpp` for the precedent.

## Tasks

| ID   | Task                                                                                                                                                                                                                                | Blocked By   | Risk | Files                                                                                              | Description                                                                                                                                                                                                                                                                                       |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------|------|----------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| T1   | Add `Config::BIDIRECTIONAL_ENABLED` (`#define`-guarded default `true`) and `Config::BIDIRECTIONAL_TURN_THRESHOLD_DEG` (`90.0f`) to `Config.h`. Place under the existing pass-through block (`Config.h:841-844`).                     | —            | low  | `WRO2026/Config.h`                                                                                 | Satisfies AC9, AC15. Mirror the `PATH_PASS_THROUGH_ENABLED_DEFAULT` pattern verbatim.                                                                                                                                                                                                             |
| T2   | Create new header `WRO2026/NavTypes.h` containing `enum class Direction : uint8_t { AUTO, FRONT, REAR }` (with Doxygen) and a single include guard. Include it from `PathTracker.h` and `Mission.h`. Standalone header avoids the circular include that would arise if `Mission.h` had to include `PathTracker.h` (which transitively pulls in `FieldMap.h` → `routes.h`) just to reference `Direction`. | —            | low  | `WRO2026/NavTypes.h` (new), `WRO2026/PathTracker.h`, `WRO2026/Mission.h`                          | Shared by builder modifiers (T8), Mission step struct (T9), and `NavEntry` struct (T8). Mirrors how `Config.h` hosts `SpeedPreset` independently of consumer headers.                                                                                                                            |
| T3   | Add `default_direction_mode` to `PipelineConfig` and `direction_override` to `EdgeInfo` (`DirectionMode` enum), emit from `header_writer.py`, regenerate `routes.h`/`routes.cpp`. Mapping: header_writer reads the graph edge attribute `direction_override` (set by T4's graph_builder) which is an integer in `{0,1,2}` corresponding to `{DEFAULT, FORCE_FORWARD, FORCE_REVERSE}`. Absence of the attribute on a graph edge → emit `DirectionMode::DEFAULT`. | T2           | med  | `tools/path_planner/pipeline_config.py`, `tools/path_planner/header_writer.py`, `WRO2026/routes.h`, `WRO2026/routes.cpp` (auto-gen) | Define DirectionMode enum (DEFAULT/FORCE_FORWARD/FORCE_REVERSE) in header_writer's emission. Append the new field at the end of EdgeInfo to keep existing field offsets stable. Run `uv run python tools/path_planner/build_routes.py` to regenerate the C++ artifacts. Satisfies AC7 prerequisite. |
| T4   | Extend Python validator and graph_builder for the `"direction"` EDGES key. Add `direction` to `_DIRECTIONAL_OPTIONAL_KEYS` in `graph_builder.py` (NOT to `_BIDIRECTIONAL_OPTIONAL_KEYS` — the override must NOT be mirrored onto the reverse edge). Validator maps `"front"` → integer 1 (FORCE_FORWARD), `"rear"` → integer 2 (FORCE_REVERSE), absence → integer 0 (DEFAULT). graph_builder stores the integer on the directed forward edge as attribute `direction_override`; T3's header_writer reads it. | T3           | med  | `tools/path_planner/field_data.py` (docs), `tools/path_planner/validator.py`, `tools/path_planner/graph_builder.py`                  | Satisfies AC7, AC8, TC8, TC15, TC16. Add a Codex P1-style note in graph_builder pointing at `_DIRECTIONAL_OPTIONAL_KEYS` and confirming `direction` is direction-dependent (asymmetric). |
| T5   | Add `_resolveDirection(...)` private helper on `PathTracker` implementing the resolution algorithm from API Contracts. Pure function on inputs (edge override, pipeline default, builder values, headings, board count, kill-switch). | T2, T3       | med  | `WRO2026/PathTracker.h`, `WRO2026/PathTracker.cpp`                                                 | Satisfies AC1–AC9 logic core. Mirror the existing `_resolveTurnMode` / `_resolveLineMode` shape (defined in the anonymous namespace at the top of `PathTracker.cpp`; called from `_onPlanNextHop` / `_dispatchEdgeAfterTurn`). Include the same heading-wrap loop pattern used at the existing wrap site in `_onPlanNextHop`. |
| T6   | Rewrite `_onPlanNextHop` to call `_resolveDirection` and branch: if FRONT, turn to `targetWorld`; if REAR, turn to `(targetWorld + 180°) mod 360` so the chassis body faces the reverse-edge. **Critical:** the `NAV_TURN_TOLERANCE_DEG` skip-turn check must compare `\|delta\|` against the *resolved* body-target, not the raw `edge.heading_deg`. Without this, when direction == REAR and the chassis happens to already face `edge.heading_deg`, the skip-turn would incorrectly fire and dispatch a forward-going LineFollower into a hop that needed REAR. Store the resolved body-heading and direction on members `_resolvedBodyHeading` and `_resolvedDirection` for `_dispatchEdgeAfterTurn` to consume. | T1, T5       | high | `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h`                                                 | Core feature — touches the FSM's central decision branch (`_onPlanNextHop`). Preserve the existing yaw-vs-peek divergence guard. **Also update the pass-through peek** (the `LINE_FOLLOW` branch that calls `FieldMap::getNextHop` to inspect the *next-next* edge) to compute body-heading delta against `_resolvedBodyHeading` of the inbound hop and a freshly-resolved body-heading of the next-next hop — NOT raw forward edge headings. See "Notes for Implementer" for why this matters in mixed REAR↔FORWARD transitions. |
| T7   | Update `_dispatchEdgeAfterTurn` to honor `_resolvedDirection`: LINE → `LineFollower.setEnabled(true).setReverse(_resolvedDirection == Direction::REAR)`, PROJECTION → same chain (`.setEnabled(true).setReverse(rev)`) + distance gate, BLIND → `Drive.forwardGyro(±power)` or `Drive.driveDist(...).setPower(±cruise)` where the sign is `+` for FRONT and `−` for REAR (per Drive's sign-by-power convention; `driveDist` rejects negative cm). Add `Fault::REVERSE_UNAVAILABLE` to the `Fault` enum and guard at the top of `_dispatchEdgeAfterTurn`: if `(_resolvedDirection == REAR) && Config::LINE_BOARD_COUNT < 2 && (edge.type == LINE \|\| edge.type == PROJECTION)`, enter fault with `from=cur`, `dest=_nextHop`. | T6           | high | `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h`                                                 | Satisfies AC1, AC10, AC11, AC12. The fault enum lives in `PathTracker.h::Fault`. **Both LINE and PROJECTION dispatch chains must add `.setReverse(...)` — PROJECTION today (`_dispatchEdgeAfterTurn`'s PROJECTION branch) does not call `setReverse`; T7 must add it.** Add the fault log line `[PathTracker] REVERSE_UNAVAILABLE: edge LINE/PROJECTION at u=<cur> v=<nxt> requires rear board`. |
| T8   | Add `setRouteDirection` and `setArrivalDirection` modifiers to `PathTrackerGoToBuilder` and stage them into the queued `NavEntry` (which T8 also extends to hold both Direction fields). Defaults are AUTO. Builder destructor copies them into the FIFO slot consumed by `_onPlanNextHop`. | T2, T5       | med  | `WRO2026/PathTracker.h`, `WRO2026/PathTracker.cpp`                                                 | Satisfies AC4, AC5, AC6, TC17. Mirror the existing `.async()` mechanism on the builder. **Blocked By T5 (in addition to T2)** so that the two tasks editing `PathTracker.h/.cpp` are serialized — T5 lands the resolver helper first, then T8 lands the builder modifiers, avoiding header merge conflicts. |
| T9   | Add `Mission::setRouteDirection` and `Mission::setArrivalDirection` modifier verbs (mutate last enqueued GOTO Step, no-op + Serial warning otherwise). Extend Step struct with two Direction fields. Step execution forwards both into PathTracker builder. **Requires T8's `PathTracker.goTo(...).setRouteDirection(...).setArrivalDirection(...)` builder methods to exist** — T9 cannot be implemented standalone. | T8           | med  | `WRO2026/Mission.h`, `WRO2026/Mission.cpp`                                                         | Satisfies AC4, AC5, AC6, TC13. Mission's GOTO step execution site (where it calls `PathTracker.goTo(...).async()`) is the one site to plumb both fields. |
| T10  | Add `chassis.reverse(distance_cm)` to `VirtualChassis` and three new primitive constants `ACT_LINE_REVERSE`, `ACT_BLIND_REVERSE`, `ACT_PROJECTION_REVERSE` to `sim/runtime.py`. Add hand-mirrored `BIDIRECTIONAL_ENABLED = True` and `BIDIRECTIONAL_TURN_THRESHOLD_DEG = 90.0` module-level constants. | T1           | med  | `tools/path_planner/sim/chassis.py`, `tools/path_planner/sim/runtime.py`                           | Satisfies AC14, TC1–TC11. Match the `ACT_LINE_PASS` pattern from `runtime.py:97`.                                                                                                                                                                                                                 |
| T11  | Implement Python `resolve_direction(...)` in `sim/runtime.py` (function-identical to T5's C++ helper — both implement the same algorithm spec from API Contracts; T11 does NOT depend on T6 compiling, only on the algorithm being specified, which the spec is). Update `PathTrackerSim.go_to` and `_should_pass_through` to call it and dispatch the matching primitive. The `_should_pass_through` peek must use resolved body-headings, not raw forward edge headings (mirrors the T6 peek update). | T10          | high | `tools/path_planner/sim/runtime.py`                                                                | Satisfies AC14, TC1–TC11, TC18. Sim primitive sequence must match `compile_path` output and the robot's `_onPlanNextHop` decisions for every fixture. **Independent of C++ tracks** — Python can land before C++ once the algorithm is locked. |
| T12  | Update `tools/path_planner/compiler.py` to emit reverse primitives based on the same resolution algorithm. `compile_path` walks NEXT_HOP, calls `resolve_direction` (T11's function), emits `("line_reverse", v, line_mode)` etc. for REAR hops. | T11          | med  | `tools/path_planner/compiler.py`                                                                   | Satisfies TC18. Compiler stays direction-agnostic at graph level (debate result), but mirrors runtime resolution at emission time so the offline sim and the robot agree. T11 transitively covers T10's dependencies. |
| T13  | Python unit tests: Codex P1-style regression that `direction` is NOT mirrored to reverse edge (TC8, TC15), schema validator tests (TC16).                                                                                            | T4           | low  | `tools/path_planner/tests/test_planner.py`, `tools/path_planner/tests/test_graph.py`               | Place TC15 next to the existing Codex P1 regression at `test_planner.py:204-244`.                                                                                                                                                                                                                  |
| T14  | Sim end-to-end tests: TC1–TC7, TC9–TC11, TC17, TC18. Add `tests/fixtures/bidirectional_field_data.py` with the routes needed.                                                                                                       | T11          | low  | `tools/path_planner/tests/test_runtime_sim.py`, `tools/path_planner/tests/fixtures/bidirectional_field_data.py` | Mirror the structure of `test_pass_through.py`. One test per AC where practical.                                                                                                                                                                                                                  |
| T15  | Pass-through interaction test: two consecutive REAR hops with collinear reverse-headings produce `ACT_LINE_PASS` (TC9).                                                                                                              | T11          | low  | `tools/path_planner/tests/test_runtime_sim.py`                                                     | Satisfies AC13. Verifies ADR-023 composition; should be a deterministic three-hop fixture.                                                                                                                                                                                                        |
| T16  | TC9-style Config parity grep tests: `Config.h` defines `BIDIRECTIONAL_ENABLED_DEFAULT` (TC14a) and `BIDIRECTIONAL_TURN_THRESHOLD_DEG` (TC14b). Sim module constants match.                                                            | T1, T10      | low  | `tools/path_planner/tests/test_pass_through.py`                                                    | Append next to existing TC9 parity tests. Same regex shape; just two new constant names.                                                                                                                                                                                                          |
| T17  | C++ host-compile test for `_onPlanNextHop` decisions: TC12 (fault on `LINE_BOARD_COUNT==1` + REAR LINE), TC13 (Mission modifier no-op on non-GOTO).                                                                                  | T7, T9       | med  | `WRO2026/test/test_path_tracker_bidirectional.cpp`                                                 | Follow the `test_path_tracker_passthrough.cpp` template. Mock LineFollower's `setReverse` call to assert direction propagation.                                                                                                                                                                    |
| T18  | Write ADR-024 (`docs/decisions/024-bidirectional-line-following.md`) documenting: precedence rule, 90° threshold rationale, why runtime (debate verdict), and the `LINE_BOARD_COUNT==1` + REAR fault behavior. Update `DECISIONS.md` index + status table. Add a one-line Roadmap entry noting #21 completion. | T7           | low  | `docs/decisions/024-bidirectional-line-following.md`, `DECISIONS.md`, `Roadmap.md`                 | 30–60 lines per ADR rules. Cite ADR-019 (line-array), ADR-023 (pass-through), ADR-012 (fluent builder) as related.                                                                                                                                                                                |

## Notes for Implementer

- **The `LineFollower::setReverse(bool)` API is fully shipped** (issue #16 / ADR-019). The wiring lives in `LineFollower.cpp::_applyEnableBuilder` (calls `_array->setReverseActive(_reverse)` when `hasReverseOverride` is set) and in the non-fluent `LineFollower::setReverse` implementation. T7 just calls it; no LineFollower-internal changes. The header docstring on `LineFollower.h::setReverse(bool)` still contains a stale "will land in T5" note from a prior plan — that work is shipped; the comment text is just outdated. The ramp-up/ramp-down sign-flip logic is also shipped — do not regress them.
- **180° turn-target for REAR.** When a hop resolves to REAR, the chassis body must face `(edge.heading_deg + 180°) mod 360` so the rear board points along the edge. `_onPlanNextHop` computes this once (stored on `_resolvedBodyHeading`) and uses it for the `Drive.turnToYaw` target AND for the `NAV_TURN_TOLERANCE_DEG` skip-turn comparison. **Failing to use the resolved body-target in the skip-turn check is the most likely correctness regression**: if the chassis happens to already face raw `edge.heading_deg` and direction == REAR, comparing `|delta|` against `edge.heading_deg` would short-circuit to "no turn needed" and dispatch a forward-going LineFollower into a REAR hop. Concrete failure mode: chassis at heading 0°, edge heading 0°, REAR resolved → must turn 180°, NOT skip the turn.
- **Pass-through with REAR uses resolved body-headings, NOT raw forward edge headings.** The pass-through peek must compute the delta between `_resolvedBodyHeading` of the inbound hop and a freshly-resolved body-heading of the next-next hop. For pure REAR/REAR with collinear forward edges this is equivalent to the raw-edge peek (adding 180° to both sides cancels) so TC9 still passes. **The fix is necessary for mixed REAR↔FORWARD transitions on anti-parallel forward edges**: e.g., a REAR hop with forward heading 0° (body heads to 180°) followed by a FORWARD hop with forward heading 180° (body stays at 180°) — body-heading delta is 0°, pass-through SHOULD fire, but raw forward-heading delta is 180° and the old peek would silently skip pass-through. TC9c covers this case.
- **The yaw-vs-peek divergence guard** in `_onPlanNextHop` must be preserved. If the peek (under the OLD body-heading) classified the next hop as pass-through but resolution now picks a different direction or body-target requiring a turn, the divergence guard's "disable follower if a turn is now required" branch handles it cleanly — same condition `delta > NAV_TURN_TOLERANCE_DEG` fires against the resolved body-target.
- **`Drive::driveDist` rejects negative cm.** Sign-by-power is the convention. T7's BLIND/PROJECTION REAR dispatch must encode reverse via `setPower(-cruise)`, never `driveDist(-cm)`.
- **PROJECTION reverse needs `setReverse(true)`.** The current PROJECTION dispatch in `_dispatchEdgeAfterTurn` chains `LineFollower.setEnabled(true).setPreset(...).setDistanceTarget(...)` without `.setReverse(...)`. T7 must add `.setReverse(_resolvedDirection == Direction::REAR)` to both the LINE and PROJECTION dispatch chains. Without this, PROJECTION REAR would still read the front board and the distance gate would face the wrong way.
- **Direction enum interchange.** `Direction::AUTO` and `Routes::DirectionMode::DEFAULT` are distinct C++ types that both encode "no override." Never compare across types — the resolution algorithm tests each independently with its own value (`Direction::AUTO` for builder fields, `DirectionMode::DEFAULT` for edge / pipeline). Stamping out cross-type comparisons is a code-review checklist item.
- **Rollback plan.** Every change is gated by `Config::BIDIRECTIONAL_ENABLED`. Setting `#define BIDIRECTIONAL_ENABLED_DEFAULT false` in a competition build disables auto entirely while keeping the manual override surfaces inert (operator who never calls `setRouteDirection` / `setArrivalDirection` sees zero behavioral change). TC14c verifies this gate path actually works.
- **CI.** GitHub Actions runs pytest and Arduino compile on every push (`.github/workflows/`). T17 host-compile test is the main C++-side coverage in CI; hardware integration is covered manually before merge.
- **Sim primitive parity (`feedback_sim_mirrors_robot`).** T10/T11/T12 are the load-bearing trio. If any divergence between robot resolution and sim resolution, TC18 fails — keep both in lockstep.
- **DO NOT** add a turn-cost penalty to `tools/path_planner/planner.py::compute_all_pairs_next_hop`. The debate verdict (3.0/10 against planner-side) explicitly rejects it on correctness grounds — turn-penalty Dijkstra requires line-graph state expansion that the current `NEXT_HOP[u][v]` schema cannot represent. The planner stays direction-agnostic.
