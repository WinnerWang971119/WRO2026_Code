# Path-Planner Modes, TurnGyro Rewrite, Intersection Overshoot, PROJECTION Restore & Naming Cleanup Plan

**Date:** 2026-05-17
**Status:** Approved — ready for execute

**Goal:** Make path-planner dispatch configurable per primitive category (line-to-intersection / blind / projection / turn), restore `EDGE_PROJECTION` lost in PR3, fix the line-sensor-to-axle offset bug at every intersection arrival, replace the lower-MCU `MiniR4.DriveDC.TurnGyro` with our own two-stage Drive.cpp state machine, plant tunables in `Config.h`, and clean up "goal" naming + `EdgeType::*_E` suffix noise — all behavior-neutral at the default so tomorrow's hardware test exercises everything without surprise.

**Approach:** Python pipeline gets a new `pipeline_config.py` whose values are baked into a generated `Routes::PIPELINE_CONFIG` struct; `field_data.py` edges accept optional `primitive_mode` / `pre_turn_mode` / `pivot` overrides that compile into per-edge fields on `Routes::EdgeInfo`. `PathTracker` looks up overrides first, falls back to `PIPELINE_CONFIG`. `Drive.cpp` gains `_updateTurnProfile` parallel to `_updateProfile`, implementing cruise → slow-zone → tolerance brake without any MiniR4 TurnGyro calls. `PathTracker` adds a `LINE_OVERSHOOT` state that re-arms `LineFollower.setDistanceTarget(LINE_SENSOR_AXLE_OFFSET_CM)` after every intersection so the wheel axle (not the sensor) lands on the node. PROJECTION returns as a first-class edge type. EdgeType drops `_E`. Internals rename `_currentGoal`→`_destinationNode`, etc.; public `goTo` verb stays.

## Scope

**In scope:**
- New `tools/path_planner/pipeline_config.py` with `PipelineConfig` dataclass + per-category mode enums (`LineMode`, `BlindMode`, `ProjectionMode`, `TurnMode`).
- Extend `field_data.py` schema: edges accept optional `primitive_mode`, `pre_turn_mode`, `pivot` fields; defaults left blank inherit from `PipelineConfig`.
- Restore `EDGE_PROJECTION` Python constant in `graph_builder.py` + validator support.
- Extend `header_writer.py`: emit `Routes::PIPELINE_CONFIG` struct + new per-edge override fields + `EdgeType::PROJECTION` enum value.
- Rename `EdgeType::LINE_E` → `LINE`, `BLIND_E` → `BLIND` in `routes.h`/`routes.cpp`/`PathTracker.cpp`/`header_writer.py` (no `_E` suffix on the new `PROJECTION`).
- Replace `MiniR4.DriveDC.TurnGyro` calls inside `Drive.cpp` with a new `_updateTurnProfile()` state machine (two-stage: cruise → slow-zone → tolerance brake). Public `Drive.turnDeg(deg).async()` / `Drive.turnToYaw(yaw).async()` builders unchanged.
- Add `LINE_OVERSHOOT` state to `PathTracker` FSM: on `Intersection.isAtIntersection()`, re-arm `LineFollower.setEnabled(true).setDistanceTarget(LINE_SENSOR_AXLE_OFFSET_CM)`, wait on `isDistanceReached()`, then advance `Localization`.
- Add `PROJECTION_FOLLOW` state to `PathTracker` FSM (dispatch `LineFollower.setEnabled(true).setDistanceTarget(edge.distance_cm)`, wait on `isDistanceReached()`).
- Wire per-edge mode override + `PIPELINE_CONFIG` fallback through `PathTracker._dispatchEdgeAfterTurn()`.
- Planner-side single-wheel pivot geometric compensation: when an edge is tagged `pivot=PIVOT_LEFT`/`PIVOT_RIGHT`, Python recomputes the following edge's `distance_cm` / `heading_deg` to land the chassis center on the next node.
- New `Config.h` constants: `LINE_SENSOR_AXLE_OFFSET_CM = 5.0f`, `TURN_CRUISE_PWM = 50`, `TURN_SLOW_ZONE_DEG = 30.0f`, `TURN_SLOW_SPEED_PWM = 20`, `TURN_TOLERANCE_DEG = 1.0f`. Bump `PATH_QUEUE_CAPACITY` from `8` to `32`.
- Rename PathTracker internals (`_currentGoal` → `_destinationNode`, `GoalEntry` → `NavEntry`, `FaultInfo.goalNode` → `destNode`). Public `goTo` verb and `Mission.goTo` DSL unchanged.
- Mirror PROJECTION + LINE_OVERSHOOT in `tools/path_planner/sim/runtime.py` so Python tests still cover the runtime FSM.
- Python unit tests for: pipeline_config emission, per-edge override emission, PROJECTION round-trip, EdgeType rename grep, idempotency, pivot compensation math, runtime-sim FSM transitions.
- Re-run `uv run python tools/path_planner/build_routes.py` to regenerate `WRO2026/routes.h` + `WRO2026/routes.cpp` with the new shape.

**Out of scope:**
- On-hardware verification — Diego handles that after the flash.
- Tuning the new `Config.h` constants to measured chassis values — placeholders only.
- `DECISIONS.md` / `Roadmap.md` updates — do those after tomorrow's hardware results confirm or contradict the design.
- New Mission DSL verbs.
- Mid-mission dynamic replanning.
- Recalibrating any PID gains.
- A `.tShape(false)` opt-out setter (already a deliberate non-feature per DECISIONS.md).

## Decisions

- **Mode config lives in Python `pipeline_config.py`, baked into routes.cpp at build time, with optional per-edge overrides in `field_data.py`.** Combines global defaults (one mode per category) with per-edge granularity. Renders into the generated artifacts so PathTracker dispatch is a table lookup, no runtime API setter needed.
- **TurnGyro rewritten in `Drive.cpp` parallel to `_updateProfile`, removes all `MiniR4.DriveDC.TurnGyro` calls.** Smallest blast radius — callers keep `Drive.turnDeg(deg)`/`Drive.turnToYaw(yaw)` API; only the executor changes. Two-stage profile: cruise at `TURN_CRUISE_PWM` until within `TURN_SLOW_ZONE_DEG` of target, ramp/snap to `TURN_SLOW_SPEED_PWM`, brake when within `TURN_TOLERANCE_DEG`.
- **Intersection overshoot fix reuses `LineFollower.setDistanceTarget()` + `isDistanceReached()`.** Already-tested infra. Adds one PathTracker state (`LINE_OVERSHOOT`); preserves P+D corrections through the overshoot.
- **PROJECTION returns as a first-class edge type + Python primitive + PathTracker FSM state.** PR3 removal was premature; the use case (waypoints with no physical intersection marker) is back on the table.
- **`EdgeType` enum drops the `_E` suffix.** Scoped enums (`enum class`) make `NodeType::LINE` and `EdgeType::LINE` non-conflicting; the suffix was redundant noise.
- **Single-wheel pivot compensation is planner-side.** Python recomputes the next edge's geometry. Runtime stays naive — fewer C++ bugs and easier to audit the math at build time.
- **Per-edge override has separate optional fields for `primitive_mode` and `pre_turn_mode`.** Both can be set independently or left blank. Most flexible composition.
- **Public `goTo` verb stays; only internals rename.** No Mission DSL churn. `_currentGoal` → `_destinationNode`, `GoalEntry` → `NavEntry`, etc.
- **Multi-hop dispatch stays.** Caller specifies only the final destination — the whole point of having a path planner.
- **`PATH_QUEUE_CAPACITY` bumped 8 → 32.** Trivial RAM cost (32 bytes), accommodates missions chaining 10+ destinations.
- **Default modes (line/blind/projection/turn) equal today's runtime behavior.** Refactor is behavior-neutral at the default; only the intersection-overshoot fix and the new TurnGyro change observable runtime behavior.

## Acceptance Criteria

- [ ] `routes.h` declares `enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2, PROJECTION = 3 };` (no `_E` suffix anywhere in repo).
- [ ] `routes.h` declares `enum class LineMode` / `BlindMode` / `ProjectionMode` / `TurnMode` each with a `DEFAULT = 0` sentinel meaning "inherit `PIPELINE_CONFIG`".
- [ ] `routes.h` declares a `Routes::PipelineConfig` struct and `extern const PipelineConfig PIPELINE_CONFIG;`; `routes.cpp` defines it with the values from `pipeline_config.py`.
- [ ] `Routes::EdgeInfo` carries optional override fields: `LineMode line_mode_override`, `BlindMode blind_mode_override`, `ProjectionMode projection_mode_override`, `TurnMode pre_turn_mode_override` — each defaulting to `DEFAULT`.
- [ ] `Config.h` includes `LINE_SENSOR_AXLE_OFFSET_CM`, `TURN_CRUISE_PWM`, `TURN_SLOW_ZONE_DEG`, `TURN_SLOW_SPEED_PWM`, `TURN_TOLERANCE_DEG` with Doxygen + `TODO: measure/tune` markers; `PATH_QUEUE_CAPACITY = 32`.
- [ ] `Drive.cpp` contains no calls to `MiniR4.DriveDC.TurnGyro`. `_updateTurnProfile()` exists and is invoked from `Drive.update()` parallel to `_updateProfile()`.
- [ ] `PathTracker.cpp` FSM has states `IDLE`, `DEQUEUE`, `PLAN_NEXT_HOP`, `TURNING`, `LINE_FOLLOW`, `LINE_OVERSHOOT` (new), `PROJECTION_FOLLOW` (new), `BLIND_FORWARD`, `FAULT`. Internals renamed (`_destinationNode`, `NavEntry`, `destNode`).
- [ ] `Mission.goTo("G")` DSL signature and `PathTracker.goTo(node)` / `PathTracker.goTo(name)` public verbs are byte-identical to today.
- [ ] Running `uv run python tools/path_planner/build_routes.py --no-viz` succeeds with exit code 0 against `field_data.py`. Re-running produces byte-identical artifacts (idempotency contract is preserved).
- [ ] `uv run pytest tools/path_planner/tests/` passes, including new tests for: pipeline_config emission, per-edge override emission, PROJECTION round-trip, idempotency, pivot compensation math, runtime-sim LINE_OVERSHOOT + PROJECTION_FOLLOW transitions.
- [ ] `Drive.turnDeg(deg)` / `turnToYaw(yaw)` fluent builders compile with no API change visible to callers; the queue dispatcher invokes the new `_updateTurnProfile` path.
- [ ] Sketch compiles in Arduino IDE without warnings.

## Data Model

### Python — new `tools/path_planner/pipeline_config.py`
```python
from dataclasses import dataclass
from enum import IntEnum

class LineMode(IntEnum):
    DEFAULT = 0   # inherit PIPELINE_CONFIG.line at runtime
    NORMAL  = 1
    SLOW    = 2
    FAST    = 3

class BlindMode(IntEnum):
    DEFAULT     = 0
    SYNC_TSHAPE = 1   # today's runtime default
    GYRO_TSHAPE = 2   # "drive heading by gyro" mode
    RAW_FLAT    = 3   # raw PWM, no profile

class ProjectionMode(IntEnum):
    DEFAULT = 0
    NORMAL  = 1
    SLOW    = 2
    FAST    = 3

class TurnMode(IntEnum):
    DEFAULT     = 0
    COUNTER_ROT = 1   # both wheels, opposite directions
    PIVOT_LEFT  = 2   # left wheel still
    PIVOT_RIGHT = 3   # right wheel still

@dataclass(frozen=True)
class PipelineConfig:
    line:       LineMode       = LineMode.NORMAL
    blind:      BlindMode      = BlindMode.SYNC_TSHAPE
    projection: ProjectionMode = ProjectionMode.NORMAL
    turn:       TurnMode       = TurnMode.COUNTER_ROT

# Mission-wide default. Edit here to swap defaults globally.
PIPELINE_CONFIG = PipelineConfig()
```

### Python — `field_data.py` schema extension
```python
EDGES = {
    ("A", "B"): {
        "type": EDGE_LINE,
        "distance_cm": 30.0,
        "heading_deg": 0.0,
        # All three optional — default DEFAULT == inherit PIPELINE_CONFIG.
        "primitive_mode": LineMode.FAST,         # this LINE edge cruises FAST
        "pre_turn_mode":  TurnMode.PIVOT_LEFT,   # the turn approaching B uses left-wheel pivot
        "pivot":          Pivot.LEFT,            # marks single-wheel pivot for compensation
    },
    ("B", "C"): {
        "type": EDGE_PROJECTION,
        "distance_cm": 12.0,                     # encoder-stopped, no intersection marker
        "heading_deg": 90.0,
    },
    ...
}
```

### C++ — `routes.h` additions
```cpp
namespace Routes {
    enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2, PROJECTION = 3 };

    enum class LineMode       : uint8_t { DEFAULT = 0, NORMAL = 1, SLOW = 2, FAST = 3 };
    enum class BlindMode      : uint8_t { DEFAULT = 0, SYNC_TSHAPE = 1, GYRO_TSHAPE = 2, RAW_FLAT = 3 };
    enum class ProjectionMode : uint8_t { DEFAULT = 0, NORMAL = 1, SLOW = 2, FAST = 3 };
    enum class TurnMode       : uint8_t { DEFAULT = 0, COUNTER_ROT = 1, PIVOT_LEFT = 2, PIVOT_RIGHT = 3 };

    struct PipelineConfig {
        LineMode       line;
        BlindMode      blind;
        ProjectionMode projection;
        TurnMode       turn;
    };
    extern const PipelineConfig PIPELINE_CONFIG;

    struct EdgeInfo {
        EdgeType       type;
        float          distance_cm;
        float          heading_deg;
        LineMode       line_mode_override;        // DEFAULT = inherit PIPELINE_CONFIG.line
        BlindMode      blind_mode_override;       // DEFAULT = inherit
        ProjectionMode projection_mode_override;  // DEFAULT = inherit
        TurnMode       pre_turn_mode_override;    // DEFAULT = inherit; applies to the turn PRECEDING this edge
    };
    // existing NODES / NEXT_HOP / EDGES tables unchanged in shape
}
```

### `Config.h` additions
```cpp
namespace Config {
    // ... existing ...

    /** @brief Distance from line-sensor centerline to wheel-axle line (cm).
     *         Positive = sensor in front of axle. @note TODO: measure on real chassis. */
    constexpr float LINE_SENSOR_AXLE_OFFSET_CM = 5.0f;

    /** @brief Custom TurnGyro cruise PWM (replaces MiniR4 lower-MCU turn). */
    constexpr int16_t TURN_CRUISE_PWM = 50;

    /** @brief Slow-zone half-width in degrees. Within ±this band of target, ramp to slow speed. */
    constexpr float   TURN_SLOW_ZONE_DEG = 30.0f;

    /** @brief Slow-zone PWM. Held until |delta| ≤ TURN_TOLERANCE_DEG. */
    constexpr int16_t TURN_SLOW_SPEED_PWM = 20;   // at deadband floor → max stop precision

    /** @brief Final tolerance in degrees. |delta| ≤ this → brake + declare done. */
    constexpr float   TURN_TOLERANCE_DEG = 1.0f;

    // Bumped 8 → 32 for missions chaining 10+ destinations.
    constexpr uint8_t PATH_QUEUE_CAPACITY = 32;
}
```

### PathTracker FSM (new states marked NEW)
```
IDLE → DEQUEUE → PLAN_NEXT_HOP →
    [turn needed?]
       Drive.turnDeg(delta, mode_from_override_or_default).async() → TURNING
    [no turn]
       → _dispatchEdgeAfterTurn()

_dispatchEdgeAfterTurn():
    LINE        → LineFollower.setEnabled(true, primitive_mode_override or PIPELINE_CONFIG.line) → LINE_FOLLOW
    PROJECTION  → LineFollower.setEnabled(true).setDistanceTarget(edge.distance_cm) → PROJECTION_FOLLOW   (NEW)
    BLIND       → Drive.driveDist(edge.distance_cm, mode=blind_override_or_default).async() → BLIND_FORWARD

LINE_FOLLOW → Intersection.isAtIntersection()?
    → LineFollower.setEnabled(true).setDistanceTarget(LINE_SENSOR_AXLE_OFFSET_CM)
    → LINE_OVERSHOOT                                                                  (NEW)

LINE_OVERSHOOT → LineFollower.isDistanceReached()?                                    (NEW)
    → LineFollower.setEnabled(false)
    → Localization.onIntersection(_nextHop)
    → PLAN_NEXT_HOP

PROJECTION_FOLLOW → LineFollower.isDistanceReached()?                                 (NEW)
    → LineFollower.setEnabled(false)
    → Localization.onDeadReckonedAdvance(_nextHop)
    → PLAN_NEXT_HOP

BLIND_FORWARD → Drive.isIdle()?
    → Localization.onDeadReckonedAdvance(_nextHop)
    → PLAN_NEXT_HOP
```

### `Drive._updateTurnProfile()` state machine
```
IDLE  →  enqueued TURN command? → init: read IMU yaw, store start, compute delta sign → CRUISE
CRUISE →
    delta = normalize(target - currentYaw)
    if |delta| ≤ TURN_TOLERANCE_DEG    → BRAKE
    if |delta| ≤ TURN_SLOW_ZONE_DEG    → SLOW
    else                                 → issue Move(±TURN_CRUISE_PWM, ∓TURN_CRUISE_PWM)   [counter-rot]
                                         or Move(±TURN_CRUISE_PWM, 0) / Move(0, ∓TURN_CRUISE_PWM)  [pivot]
SLOW →
    if |delta| ≤ TURN_TOLERANCE_DEG    → BRAKE
    else                                 → issue Move at TURN_SLOW_SPEED_PWM (same wheel-pattern)
BRAKE → Move(0,0); brake(true); → IDLE; _turnDone = true
```

## Error Handling

- **TurnGyro never converges** (e.g., IMU stuck): caught by `Drive._drainBlocking`'s `DRIVE_DRAIN_TIMEOUT_MS_PER_CMD` deadline → latches `Fault::DRAIN_TIMEOUT`. PathTracker's own drain budget catches the propagation.
- **`LineFollower.isDistanceReached()` never fires during LINE_OVERSHOOT or PROJECTION_FOLLOW** (encoder dead): same `Drive._drainBlocking` deadline; PathTracker's `PATH_DRAIN_TIMEOUT_MS_PER_GOAL` is the outer net.
- **`DEFAULT` sentinel in per-edge override**: PathTracker resolves to `PIPELINE_CONFIG.<category>` (single helper `_resolveMode(EdgeInfo, category)`).
- **Edge tagged `pivot=PIVOT_*` but planner can't compute compensation** (e.g., degenerate 0° turn, missing track width): `build_routes.py` fails with exit code 2 and a descriptive error.
- **Idempotency regression after rename**: golden tests + byte-diff assertion in `test_header_writer.py` catches at CI time, before flash.
- **`EDGE_PROJECTION` cycle without intersection** (badly shaped graph): validator catches at build time.

## Testing Strategy

**Levels:** Unit (Python), Integration (Python runtime sim), grep (rename sweeps).

| ID  | Test Case | Type | Expected |
|-----|-----------|------|----------|
| TC1 | Default `PipelineConfig()` emits canonical struct in routes.cpp | Unit | `PIPELINE_CONFIG = { LineMode::NORMAL, BlindMode::SYNC_TSHAPE, ProjectionMode::NORMAL, TurnMode::COUNTER_ROT };` |
| TC2 | Edge with `primitive_mode=LineMode.FAST` emits override field | Unit | `EdgeInfo` cell has `LineMode::FAST` instead of `LineMode::DEFAULT` |
| TC3 | Edge with `pre_turn_mode=TurnMode.PIVOT_LEFT` emits override field | Unit | `EdgeInfo` cell has `TurnMode::PIVOT_LEFT` |
| TC4 | PROJECTION edge round-trips: field_data → graph → routes.cpp → compiler primitive | Unit | `("follow_line_to_distance", cm)` primitive emitted; `EdgeType::PROJECTION` cell present |
| TC5 | EdgeType rename sweep: zero occurrences of `LINE_E`/`BLIND_E` anywhere | grep | 0 matches in `tools/`, `WRO2026/`, `docs/` |
| TC6 | Idempotency: re-running `build_routes.py` on unchanged inputs produces byte-identical routes.h/cpp | Integration | Files unchanged on second invocation |
| TC7 | Pivot compensation: 0° turn → zero displacement | Unit | Following edge geometry unchanged |
| TC8 | Pivot compensation: 90° pivot (TRACK_MM=150) → chord-length = 21.21cm displacement at perpendicular bisector heading | Unit | Adjusted distance/heading matches hand-derived geometry within 0.01 cm / 0.1° |
| TC9 | Runtime-sim LINE_OVERSHOOT: chassis advances by edge.distance + `LINE_SENSOR_AXLE_OFFSET_CM` total | Integration | Sim chassis position delta = edge.distance + offset |
| TC10 | Runtime-sim PROJECTION_FOLLOW: chassis advances by edge.distance and Localization advances via `onDeadReckonedAdvance` | Integration | Position correct; `current_node == nxt` after step |
| TC11 | Per-edge override resolution: `LineMode.DEFAULT` → falls back to `PIPELINE_CONFIG.line` in dispatch | Unit (mirror in sim) | Sim chassis uses default mode when override == DEFAULT |
| TC12 | Drive.cpp grep: zero `MiniR4.DriveDC.TurnGyro(` calls | grep | 0 matches |
| TC13 | Public Mission.goTo("G") DSL: WRO2026.ino unchanged in shape | grep | No diff in Mission.goTo lines |

**Test data:** Reuse `tools/path_planner/tests/golden/routes_toy.{h,cpp}`. Extend a sibling fixture with: one PROJECTION edge, one edge with `primitive_mode` override, one edge with `pivot=LEFT`.

**Run command:** `uv run pytest tools/path_planner/tests/` + `uv run python tools/path_planner/build_routes.py --no-viz`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Add new Config.h constants and bump PATH_QUEUE_CAPACITY | — | low | `WRO2026/Config.h` | Add `LINE_SENSOR_AXLE_OFFSET_CM=5.0f`, `TURN_CRUISE_PWM=50`, `TURN_SLOW_ZONE_DEG=30.0f`, `TURN_SLOW_SPEED_PWM=20`, `TURN_TOLERANCE_DEG=1.0f` with full Doxygen + TODO-tune markers. Change `PATH_QUEUE_CAPACITY` from 8 to 32 and update its docstring. Satisfies AC bullet on Config.h constants. |
| T2 | Create `tools/path_planner/pipeline_config.py` | — | med | `tools/path_planner/pipeline_config.py` | New file. Define `LineMode`, `BlindMode`, `ProjectionMode`, `TurnMode` IntEnums each starting at `DEFAULT=0`. Define `PipelineConfig` dataclass with category fields + canonical defaults (NORMAL / SYNC_TSHAPE / NORMAL / COUNTER_ROT). Export `PIPELINE_CONFIG = PipelineConfig()`. Satisfies AC bullet on PIPELINE_CONFIG. |
| T3 | Restore `EDGE_PROJECTION` + extend edge schema in graph_builder.py | — | med | `tools/path_planner/graph_builder.py` | Add `EDGE_PROJECTION = "projection"` constant alongside `EDGE_LINE`/`EDGE_BLIND`. Allow edges with `type=EDGE_PROJECTION` through validation. Accept optional `primitive_mode`/`pre_turn_mode`/`pivot` keys on edge dicts and forward them to the nx.DiGraph as edge attributes. Pivot field uses a new `Pivot` IntEnum (`NONE=0, LEFT=1, RIGHT=2`) — declare it here. Satisfies AC bullet on PROJECTION first-class. |
| T4 | Update field_data.py with example projection + override edge | T3, T2 | low | `tools/path_planner/field_data.py` | Add at least one PROJECTION edge and one edge with `primitive_mode` override + one with `pivot=Pivot.LEFT` so the regen exercises the new emit paths. Keep existing nodes/edges intact. |
| T5 | Implement single-wheel pivot compensation in planner.py | T3, T4 | high | `tools/path_planner/planner.py` | When an edge is tagged `pivot=LEFT` or `pivot=RIGHT`, compute the chassis-center displacement (chord of radius `TRACK_MM/2 / 10 = TRACK_CM/2`) and adjust the FOLLOWING edge's `distance_cm` + `heading_deg` so the chassis center lands on the next node. Use `Config::TRACK_MM` (re-declare as Python const since field_data.py shouldn't depend on C++). Add unit-test-friendly pure helper `compute_pivot_compensation(turn_deg, track_cm, next_edge_dist, next_edge_heading) -> (adj_dist, adj_heading)`. Satisfies AC bullet on pivot compensation. |
| T6 | Extend validator.py for new schema | T3, T2 | med | `tools/path_planner/validator.py` | Accept `EDGE_PROJECTION` edges; require `distance_cm` and `heading_deg`. Validate optional mode override fields against the IntEnum constants. Validate `pivot` field. Reject degenerate pivot configs (turn_deg too small for compensation to be meaningful). Surface errors with the existing `FieldDataError`. |
| T7 | Rewrite header_writer.py: PIPELINE_CONFIG struct, new EdgeInfo fields, EdgeType rename, PROJECTION | T2, T3 | high | `tools/path_planner/header_writer.py` | (a) Add `_EDGE_TYPE_TO_ENUM` entry for PROJECTION. (b) Rename `LINE_E`→`LINE`, `BLIND_E`→`BLIND` in the emit map and update enum-declaration emit line. (c) Emit `LineMode`/`BlindMode`/`ProjectionMode`/`TurnMode` enum class declarations. (d) Emit `struct PipelineConfig {...}` + `extern const PipelineConfig PIPELINE_CONFIG;` in routes.h. (e) Emit `const PipelineConfig PIPELINE_CONFIG = {...};` in routes.cpp populated from `pipeline_config.PIPELINE_CONFIG`. (f) Extend `_edge_cell` to emit four override fields (DEFAULT when not specified on the edge). Preserve idempotency contract: same sort order, same float formatting, same line endings. Satisfies AC bullets on routes.h enums, PIPELINE_CONFIG, EdgeInfo overrides, EdgeType rename. |
| T8 | Extend compiler.py: PROJECTION primitive + per-edge mode in Primitive tuples | T3, T2 | med | `tools/path_planner/compiler.py` | Add `ACT_FOLLOW_DIST = "follow_line_to_distance"` constant. Emit `(ACT_FOLLOW_DIST, distance_cm)` for `EDGE_PROJECTION` edges. Optionally extend `Primitive` to carry per-edge mode metadata (for sim tests only). Update docstring to reflect the restored PROJECTION primitive. |
| T9 | Mirror restored PROJECTION + LINE_OVERSHOOT + per-edge modes in sim/runtime.py | T8 | med | `tools/path_planner/sim/runtime.py`, `tools/path_planner/sim/chassis.py` | Add `follow_line_to_distance(cm)` handling to `PathTrackerSim.go_to`/`step_through` (advance chassis by cm; call `on_dead_reckoned_advance`). Add LINE_OVERSHOOT step after each LINE edge that advances chassis by `LINE_SENSOR_AXLE_OFFSET_CM`. Add a `pipeline_config` reference + per-edge override resolution helper. Chassis class gains `follow_line_to_distance(cm)` method that just calls `forward(cm)`. |
| T10 | Wire pipeline_config through build_routes.py | T2, T7 | low | `tools/path_planner/build_routes.py` | Import `pipeline_config.PIPELINE_CONFIG` and pass it into `emit_routes(...)`. No CLI flag change. |
| T11 | Add Python tests for pipeline_config + new EdgeInfo emit | T7, T10 | med | `tools/path_planner/tests/test_header_writer.py`, `tools/path_planner/tests/golden/routes_toy.{h,cpp}` | Add TC1, TC2, TC3 from the testing matrix. Regenerate the golden fixture once with the new shape (this is a one-off authorized rebaseline, since the schema changed); subsequent runs MUST be byte-identical. Add TC5 grep test (`assert "LINE_E" not in <repo>/tools/...`). |
| T12 | Add Python tests for PROJECTION compile path + sim FSM | T8, T9 | med | `tools/path_planner/tests/test_compiler.py`, `tools/path_planner/tests/test_runtime_sim.py` | Add TC4 (PROJECTION compile → primitive), TC9 (sim LINE_OVERSHOOT), TC10 (sim PROJECTION_FOLLOW), TC11 (override resolution). Reuse the extended golden fixture. |
| T13 | Add Python tests for pivot compensation math | T5 | med | `tools/path_planner/tests/test_planner.py` | Add TC7 (0° turn = no displacement), TC8 (90° pivot with TRACK_CM=15 = chord 21.21cm). Use the pure helper `compute_pivot_compensation` from T5 — no graph dependency. |
| T14 | Regenerate `WRO2026/routes.h` + `routes.cpp` via build_routes.py | T4, T6, T7, T8, T10, T11 | low | `WRO2026/routes.h`, `WRO2026/routes.cpp` | Run `uv run python tools/path_planner/build_routes.py --no-viz`. Commit the regenerated files. Verify byte-identical re-emit by running twice. |
| T15 | Add `_updateTurnProfile` state machine in Drive.h header | T1 | high | `WRO2026/Drive.h` | Add private state: `enum class TurnProfileState { IDLE, CRUISE, SLOW, BRAKE };`, `_turnProfileState`, `_turnTargetYaw`, `_turnDeltaSign`, `_turnMotorMode` (counter-rot or pivot-side). Add `_updateTurnProfile()` private method declaration. NO public API change. |
| T16 | Implement `_updateTurnProfile` in Drive.cpp; remove MiniR4 TurnGyro calls | T15 | high | `WRO2026/Drive.cpp` | Rewrite `_execTurnDeg` and `_execTurnToYaw` to seed `_turnProfileState=CRUISE` + compute target yaw + sign, NEVER call `MiniR4.DriveDC.TurnGyro` again. Add `_updateTurnProfile()` implementing the four-state machine from the Data Model section: read IMU yaw, compute normalized delta, dispatch wheel pattern by `_turnMotorMode`, transition through CRUISE → SLOW → BRAKE. On entering BRAKE: `Move(0,0)`, `brake(true)`, set `_turnDone=true`, return to IDLE. Hook the new method into `Drive.update()` parallel to `_updateProfile()` (call ordering: drain task-done bit → `_updateProfile` → `_updateTurnProfile` → `_dispatchNext`). Resolve `_turnMotorMode` from `cmd.turnMode` (existing `DriveCommand` field, repurposed: 0=counter-rot, 1=pivot-left, 2=pivot-right). Satisfies AC bullets on Drive.cpp / `_updateTurnProfile` / zero MiniR4.TurnGyro calls. |
| T17 | PathTracker rename internals + add LINE_OVERSHOOT + PROJECTION_FOLLOW states + mode resolution | T14, T16 | high | `WRO2026/PathTracker.h`, `WRO2026/PathTracker.cpp` | (a) Rename `_currentGoal` → `_destinationNode`, `GoalEntry` → `NavEntry`, `FaultInfo.goalNode` → `destNode`. Public verbs untouched. (b) Add `LINE_OVERSHOOT` + `PROJECTION_FOLLOW` to `State` enum. (c) In `_dispatchEdgeAfterTurn`: handle `EdgeType::PROJECTION` by calling `LineFollower.setEnabled(true).setDistanceTarget(edge.distance_cm)` and setting state `PROJECTION_FOLLOW`. (d) Modify `LINE_FOLLOW` case in `update()` to transition to `LINE_OVERSHOOT` instead of straight to `PLAN_NEXT_HOP` — re-arms `LineFollower` with `setDistanceTarget(Config::LINE_SENSOR_AXLE_OFFSET_CM)`. (e) Add `LINE_OVERSHOOT` case in `update()` that waits on `LineFollower.isDistanceReached()` then advances Localization and transitions to PLAN_NEXT_HOP. (f) Add `PROJECTION_FOLLOW` case similarly (calls `Localization.onDeadReckonedAdvance`). (g) In `_onPlanNextHop`: resolve `pre_turn_mode_override` from edge → fall back to `PIPELINE_CONFIG.turn`; pass into `Drive.turnDeg(delta).setMode(modeByte).async()` (since builder takes `uint8_t mode`, map `TurnMode::*` → the byte the new `_updateTurnProfile` expects). (h) Resolve `primitive_mode_override` per category for LINE/BLIND/PROJECTION dispatch. Add private helper `_resolveLineMode(EdgeInfo)`, `_resolveBlindMode(EdgeInfo)`, etc. Satisfies AC bullets on FSM new states, renamed internals, public verb unchanged. |
| T18 | Verify LineMode → SpeedPreset mapping in T17 resolution helper | T1 | low | (no source files touched) | Verify in code review that LineFollower's existing `setPreset(Config::SpeedPreset)` covers SLOW/NORMAL/FAST and that the resolution helper in T17 maps `LineMode::SLOW`→`SpeedPreset::SLOW`, etc. Document the mapping in a comment block at the top of the resolution helper. If a mismatch is found, fix it in T17 (this task is verification only). |
| T19 | Sketch sanity-compile in Arduino IDE | T14, T17 | med | (no source files touched) | Open `WRO2026/WRO2026.ino` in Arduino IDE (Arduino UNO R4 WiFi), Verify, capture any warnings to a follow-up issue. Not a hardware test — compile-only sanity check before tomorrow's flash. |

Task graph: T1, T2, T3 can run in parallel. T4 waits for T2+T3. T5/T6 wait for T3/T4. T7 waits for T2+T3. T8 waits for T3+T2. T9 waits for T8. T10 waits for T2+T7. T11 waits for T7+T10. T12 waits for T8+T9. T13 waits for T5. T14 (regen) waits for everything that touches Python (T4, T6, T7, T8, T10, T11). T15 (Drive.h) waits for T1. T16 waits for T15. T17 waits for T14 + T16. T18, T19 are tail-end verification.

## Notes for Implementer

- **EdgeType rename is a wide grep** — sweep `tools/`, `WRO2026/`, `docs/` for `LINE_E`, `BLIND_E`. The PR3-deferred doc (`docs/PHASE2_DEFERRED.md`) may reference them; leave doc strings about historical PR3 alone but update any active code references.
- **Idempotency contract is sacred** — `test_header_writer.py` has byte-diff assertions. The one-off rebaseline in T11 is the only authorized golden-fixture update; subsequent runs MUST produce identical bytes.
- **`DriveCommand.turnMode` field repurposing** — today it's pre-cast to the MiniR4 library's `uint8_t mode` (0=pivot, 1=counter). We're remapping: 0=counter-rot (new default), 1=pivot-left, 2=pivot-right. Backward-incompatible at the byte level but contained in PathTracker dispatch + the `_updateTurnProfile` consumer. Document the new mapping in a comment on `DriveCommand`'s `turnMode` field declaration.
- **`_updateTurnProfile` motor-pattern choice** — for counter-rot, drive `Move(+PWM, -PWM)`. For pivot-left, `Move(0, -PWM)`. For pivot-right, `Move(+PWM, 0)`. Signs flip based on `_turnDeltaSign` (clockwise vs counter-clockwise rotation).
- **Where to put PROJECTION_FOLLOW handler in PathTracker switch** — keep the case ordering parallel to LINE_FOLLOW for readability.
- **LineFollower's `setDistanceTarget` already auto-disables on hit and latches `_distanceReached`** — PathTracker reads it once and clears. No new LineFollower API needed.
- **Pivot compensation math** — for a single-wheel pivot through angle θ around the stationary wheel: chassis center moves along a chord of length `2 * (TRACK/2) * sin(θ/2)` at angle perpendicular to bisector of the turn. The following edge's `distance_cm` reduces by the projection of the chord onto the next-edge heading; `heading_deg` is unchanged (the planner emits absolute world headings, so the next edge's heading is independent of how we got there).
- **Rollback plan** — if tomorrow's hardware test reveals issues, the changes can be reverted by branch: `git checkout main -- WRO2026/` for the C++ side and `git checkout main -- tools/path_planner/` for the Python side. Config.h constants are additive (placeholder values), so reverting just `routes.h/cpp` would temporarily leave them unused, which is benign.
