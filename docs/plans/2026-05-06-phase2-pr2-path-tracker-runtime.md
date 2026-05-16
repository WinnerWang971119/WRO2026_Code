# Phase 2 PR 2 — Path Tracker R4 Runtime

**Status:** Pre-approved 2026-05-06 (Diego, in `/plan` session). Autonomous execution authorized.

**Goal:** Land the R4-side runtime that consumes `Routes::` from PR1 — `FieldMap`, `Localization`, `PathTracker` — plus a new `setDistanceTarget` extension on `LineFollower` so the projection-edge primitive works, all wired into a `NAV_DEMO_MODE` block in `WRO2026.ino`. Add a pure-Python end-to-end simulator under `tools/path_planner/sim/` that exercises the toy graph from PR1 against a virtual chassis and asserts arrival within tolerance — this is the **only** verification step (no hardware required) and graduates the deferred Phase 2.5 simulator entry into shipped code.

**Approach:** Add three new C++ modules (`FieldMap.h` header-only; `Localization.h/.cpp` and `PathTracker.h/.cpp` as `.h/.cpp` singletons matching the Phase 1 module shape). Extend the existing `LineFollowerEnableBuilder` with one new setter (`setDistanceTarget(cm)`) and a small encoder gate inside `LineFollowerClass::update()` — no other Phase 1 module changes. PathTracker owns a non-blocking FSM driven from `cooperativeTick()` and exposes `goTo(node)` as a fluent builder with an internal FIFO goal queue (sync-blocking by default with `.async()` opt-out, matching `Drive`'s established convention). All turns dispatch via `Drive.turnToYaw(...).async()` using a yaw offset captured at start-button press (not at `begin()`). Bump the `Routes::GoalApproach` schema by one field (`goal_node`) so PathTracker can map a goal node id to its blind-moves entry without a parallel array — regenerate `routes.h/cpp` accordingly. Replace Phase 1's `DEMO_MODE` smoke with a `NAV_DEMO_MODE` that runs `goTo("C")` then `goTo("G")` on `BTN_UP`.

## Phase 2 Delivery Context

This is **PR 2 of 3** for Phase 2. The full split (from PR1's plan):

| PR | Branch | Scope | Verifiable on |
|----|--------|-------|---------------|
| PR 1 | `phase2/path-planner-python` | `tools/path_planner/` build pipeline. Toy graph fixture. No chassis dependency. | Laptop only — **shipped** 2026-05-05 |
| **PR 2 (this spec)** | `phase2/path-tracker-runtime` | R4 runtime (`FieldMap`, `Localization`, `PathTracker`, `LineFollower.setDistanceTarget`) wired into `WRO2026.ino`. Same toy graph as PR 1. | **Laptop only** (Python simulator); hardware run deferred |
| PR 3 | `phase2/field-2026-and-mission` | Real 2026 field encoded; Serial-console mission entrypoint. Reduced-scale print test runs. | Robot + reduced-scale field |

PR 2 was originally scoped to be hardware-verified ("Robot + toy printout"); per Diego's 2026-05-06 direction it ships verified entirely on the laptop via a Python simulator. Hardware bring-up of PR 2 happens in a follow-up bench session once mechanical is ready, and any fixes from that session land as PR 2.5 patches.

## Architecture Note

`docs/PATH_PLANNER_SPEC.md` §5.1 sketched the R4 state as `(uint8_t current_node, uint8_t current_heading)` with a discrete `N=0/E=1/S=2/W=3` heading enum. PR 2 implements **continuous float degrees in `[0, 360)`** to match the Python pipeline (PR 1 already chose float degrees over the cardinal enum). The spec will be reconciled in T11 along with the PHASE2_DEFERRED bookkeeping bump.

Heading is reconciled to the IMU's frame at runtime via a one-time **operator-relative offset** captured the moment the operator presses the start button — *not* at `Localization::begin()`, because the chassis is rarely at the right pose when `MiniR4.begin()` runs. This is a project-wide convention that should generalise to any future calibration snapshot (color baselines, line thresholds, etc.).

## Scope

- **In scope:**
  - `WRO2026/FieldMap.h` — header-only `FieldMap` namespace of inline accessors over `Routes::`. No state, no `begin()`, no `extern`.
  - `WRO2026/Localization.h` + `.cpp` — `LocalizationClass` singleton holding `currentNode` and `yawOffsetDeg`. Single writer for both.
  - `WRO2026/PathTracker.h` + `.cpp` — `PathTrackerClass` singleton with non-blocking `update()`, fluent `goTo(node)` builder, internal FIFO goal queue, fault state, and per-edge FSM.
  - Extension to `WRO2026/LineFollower.h` + `.cpp`: one new builder setter `setDistanceTarget(cm)` and a small encoder-distance gate inside `update()` that auto-disables (with active brake) on hit. **No** intersection-stop setter — PathTracker remains the single read-and-clear consumer of `Intersection::isAtIntersection()` per the Phase 1 F6 contract.
  - `WRO2026/Config.h` additions: PathTracker queue capacity, NAV turn tolerance, NAV drain timeout, distance-target encoder cm-resolution.
  - `WRO2026/EventTrace.h` + `.cpp` — three new `pushEvent` callsites for `NAV ZERO` / `NAV ARR <id>` / `NAV FAULT <code>`. (Buffer capacity unchanged.)
  - `WRO2026/WRO2026.ino` — replace `DEMO_MODE` with `NAV_DEMO_MODE`, add `Localization`/`PathTracker` to the cooperative tick chain, register the cooperative tick callback into PathTracker (mirrors what Drive already has), wire BTN_UP rising-edge to `Localization.zeroHeadingFromCurrent()` followed by `goTo("C"); goTo("G");`.
  - **Schema bump on `Routes::GoalApproach`:** add `uint8_t goal_node;` field. Update `tools/path_planner/header_writer.py`, the golden fixture `tools/path_planner/tests/golden/routes_toy.h`, and regenerate `WRO2026/routes.h` + `WRO2026/routes.cpp`. PR1 explicitly said the schema is not frozen.
  - `tools/path_planner/sim/` — pure-Python simulator package: `runtime.py` (FSM mirror), `chassis.py` (virtual `(x, y, heading_deg)` with primitive transitions), `__init__.py`. Imports `field_data.py` / `graph_builder.py` / `planner.py` / `compiler.py` directly (does **not** parse `routes.h`).
  - `tools/path_planner/tests/test_runtime_sim.py` — pytest suite covering: every reachable from→goal pair on the toy graph; unreachable-goal fault path; sim/compiler primitive-equivalence on 50 random walks; chassis arrival within 0.1 cm tolerance.
  - `docs/PHASE2_DEFERRED.md` — graduate the "End-to-end pure-Python simulator" entry to shipped; keep `wall_zero`, calibration nodes, per-scenario tables, and Serial-console as deferred.
  - `docs/plans/2026-05-06-phase2-pr2-path-tracker-runtime.md` — this spec.

- **Out of scope:**
  - **Real 2026 field data** in `field_data.py` — PR 3.
  - **Serial-console mission entrypoint** — PR 3.
  - **Hardware verification** — deferred to a follow-up bench session (any fixes ship as PR 2.5 patches).
  - **Calibration nodes / `wall_zero` primitive / per-scenario NEXT_HOP tables** — Phase 2.5; remain in `PHASE2_DEFERRED.md`.
  - **IMU drift compensation** — deferred to PR 3 (where the longer real-field path exposes it).
  - **`Drive` / `Intersection` modifications** beyond what's already there.
  - **Manipulator** — the Phase 1 stub stays put; PathTracker does not call into it.
  - **Lost-line / missed-intersection recovery branches** — Phase 4 per Roadmap §7.

## Decisions

- **Heading model:** absolute turns, IMU-anchored. Localization stores `yawOffsetDeg` captured when `zeroHeadingFromCurrent()` is called (rising edge of BTN_UP). World-frame heading is `normalize(IMU_yaw - yawOffsetDeg)` in `[0, 360)`. PathTracker computes `imu_target = world_target + yawOffsetDeg` and dispatches `Drive.turnToYaw(int16_t(imu_target)).async()`. **Why:** each turn re-anchors to IMU truth — single-turn precision bounds the error rather than N × per-turn float drift. Library docs confirm `TurnGyro` accepts `async=true`; if firmware doesn't honor it the cooperative tick freezes briefly during turns (acceptable for PR2 — chassis is stationary anyway).
- **Localization writers:** Localization is the only writer of `currentNode` and `yawOffsetDeg`. PathTracker calls `onIntersection(advanceTo)` after a LINE edge arrival, `onDeadReckonedAdvance(advanceTo)` after a PROJECTION or BLIND edge arrival. Heading is *read live* (IMU + offset) — never written by PathTracker, since absolute turns make the IMU the truth.
- **PathTracker FSM states:** `IDLE → DEQUEUE → PLAN_NEXT_HOP → TURNING → (LINE_FOLLOW | PROJECTION_FOLLOW | BLIND_FORWARD) → PLAN_NEXT_HOP …` until `currentNode == goal`, then `→ GOAL_BLIND_DISPATCH → GOAL_BLIND_AWAIT → DEQUEUE`. `FAULT` is a terminal sink reachable from anywhere; brakes the chassis, clears the queue, latches `_fault`, requires `clearFault()` to re-arm.
- **Goal queue capacity:** `Config::PATH_QUEUE_CAPACITY = 8` (matches `Config::DRIVE_QUEUE_CAPACITY`).
- **goTo() builder semantics (mirrors `Drive`):** sync-blocking by default — destructor calls `_drainBlocking()` which spins `delay(2)` + the registered cooperative tick callback until `_qCount == 0 && _state == IDLE`. `.async()` opts out — destructor enqueues and returns. `goTo(uint8_t id)` and `goTo(const char* name)` overloads; name lookup is O(N) over `Routes::NODES[i].name` (toy N=7, fine at human-issue rate).
- **goTo before zero is a fault:** `zeroHeadingFromCurrent()` sets `_zeroed = true`; goTo's builder destructor checks this on enqueue and faults `Fault::NOT_ZEROED` if false. Surfaces immediately at the call site rather than at FSM dispatch time.
- **goTo to currentNode is a no-op:** if `_currentGoal == Localization.currentNode` at `PLAN_NEXT_HOP` entry, FSM transitions directly to `GOAL_BLIND_DISPATCH` if the node is a goal, else `IDLE` (or next dequeue). No spurious turns.
- **Unreachable goal:** if `Routes::NEXT_HOP[from][goal] == 0xFF`, FSM transitions to `FAULT(UNREACHABLE)` with from/to recorded in `_fault`. EventTrace pushes `NAV FAULT UNR`. Drive is stopped via `Drive.stop(true)`.
- **Drive fault propagation:** on every state-machine tick that reads `Drive.isIdle()`, also check `Drive.hasFault()`. If true, transition to `FAULT(DRIVE_FAULT)` and surface.
- **LineFollower distance gate:** the existing `LineFollowerEnableBuilder` gains exactly one new setter `setDistanceTarget(float cm)`. When set on a `setEnabled(true)` build, `_applyEnableBuilder` captures `_distanceTargetActive = true`, `_distanceTargetCm = cm`, `_distanceStartEncL = Drive.getEncL()`, `_distanceStartEncR = Drive.getEncR()`. On every tick `update()` evaluates `delta_cm = encDegToCm(avg(|getEncL() - _distanceStartEncL|, |getEncR() - _distanceStartEncR|))`; if `delta_cm >= _distanceTargetCm`, calls itself the equivalent of `setEnabled(false)` (active brake by default), clears `_distanceTargetActive`, sets a one-shot `_distanceReached` flag readable via a new `bool isDistanceReached()` getter (read-and-clear, mirroring Intersection's edge consumption).
- **No setStopAtIntersection on LineFollower:** PathTracker polls `Intersection::isAtIntersection()` directly in the LINE_FOLLOW state. Adding it to LineFollower would double-consume the edge with PathTracker (Intersection.h F6 split).
- **Schema bump on `GoalApproach`:** add `uint8_t goal_node` field after `entry_node`. Header writer emits the field; runtime maps `goalNodeId → GoalApproach idx` via O(N_GOALS) linear scan in `FieldMap::getGoalApproach(uint8_t goalId)` (returns `nullptr` if no match). N_GOALS ≤ 1 on toy, ≤ 10 on real field — fine at runtime cost.
- **Cooperative tick order:** `Drive → LineFollower → Intersection → Manipulator → PathTracker → EventTrace`. PathTracker is *after* Intersection so its read-and-clear sees the edge fired this same tick. EventTrace is last so its events latch after every other module's state has updated.
- **EventTrace events:** `NAV ZERO` (on zeroHeadingFromCurrent), `NAV ARR <name>` (on goal arrival, name truncated to 4 chars to fit the tracer width), `NAV FAULT <code>` (NOT_ZEROED → NZ, UNREACHABLE → UNR, DRIVE_FAULT → DRV, BAD_NODE → BAD).
- **Branch:** `phase2/path-tracker-runtime` (off `main`).
- **`Drive` and `Intersection` are not modified.** Auto-disable of LineFollower in `Drive._dispatchNext` already does the right thing for the new flow.

## Acceptance Criteria

- [ ] **AC1** `uv run python tools/path_planner/build_routes.py` succeeds on the toy graph after the GoalApproach schema bump; emits a fresh `WRO2026/routes.h` + `WRO2026/routes.cpp` containing the new `goal_node` field; output is byte-identical on a second run.
- [ ] **AC2** `uv run pytest tools/path_planner/tests/ -v` passes — all PR 1 tests, the updated golden `routes_toy.h` golden test, and the new `test_runtime_sim.py` suite. Total runtime under 10 s on Diego's Windows machine.
- [ ] **AC3** `tools/path_planner/sim/runtime.py` exposes a `PathTrackerSim` class that, given `field_data` + a `VirtualChassis`, executes a sequence of `goTo(node)` calls and produces the **exact same** primitive sequence that `tools.path_planner.compiler.compile_path` produces for the corresponding NEXT_HOP-walked path. Verified on every reachable from→goal pair plus 50 random multi-step sequences.
- [ ] **AC4** Simulator's virtual chassis lands within `0.1 cm` of the goal node's `(x, y)` for every reachable pair on the toy graph; chassis heading after a goal-approach blind sequence matches the expected post-blind heading from compiler.py.
- [ ] **AC5** Simulator detects `Routes::NEXT_HOP[from][goal] == 0xFF` and raises a `RuntimeError` with `from_node` and `to_node` in the message; test asserts the same fault path the C++ FSM will take.
- [ ] **AC6** C++ side: every new `.h` / `.cpp` and the modified `LineFollower.h/.cpp` + `WRO2026.ino` + `EventTrace.cpp` parse cleanly. Verification path: run `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`. If `arduino-cli` is not installed, write a compile-attempt log to `docs/phase2-pr2-compile-audit.md` listing every file, every public method, every constructor signature, and an explicit "// reviewed manually for: declared-but-undefined / extern mismatch / missing include" line per module.
- [ ] **AC7** `LocalizationClass::zeroHeadingFromCurrent()` is the *only* path that sets `_yawOffsetDeg` and `_zeroed`. Calling `goTo()` while `!isZeroed()` produces `Fault::NOT_ZEROED`, the chassis is not commanded, EventTrace receives `NAV FAULT NZ`, and the goTo's builder destructor returns without enqueueing. Verified by a host-build unit test (compiled against a tiny `Arduino.h` shim — see Notes for Implementer) **OR**, if the C++ host shim is non-trivial, the same condition is verified in the Python simulator via a `Localization` mirror class.
- [ ] **AC8** `PathTrackerClass::update()` issues *no* sync-blocking Drive or LineFollower verbs. Every Drive call from PathTracker is `.async()` (audited via grep on `PathTracker.cpp`). Every LineFollower call from PathTracker either uses the staged-builder pattern (and the builder is allowed to apply state synchronously inside `_applyEnableBuilder`) or is the new `isDistanceReached()` poll.
- [ ] **AC9** `WRO2026.ino` has *no* `DEMO_MODE` block; the only conditional demo is `NAV_DEMO_MODE` and it is gated by `_setupFailed` (motion-suppress on boot fail). The cooperative tick chain order is `Drive → LineFollower → Intersection → Manipulator → PathTracker → EventTrace`.
- [ ] **AC10** `docs/PHASE2_DEFERRED.md` no longer lists "End-to-end pure-Python simulator" under deferred; it remains in "Phase 2.5 graduates" history at the top of the file. `wall_zero`, calibration nodes, per-scenario tables, Serial console all remain deferred with their original text.
- [ ] **AC11** Branch `phase2/path-tracker-runtime` opened off `main`; commits land per task with messages following the existing `chore|feat(scope): T<n> — <summary>` style; nothing pushed to `main`.
- [ ] **AC12** No C++ files modified outside the listed in-scope set. No PR1 Python files modified except `tools/path_planner/header_writer.py` and the golden fixture (the schema-bump touch).

## Data Model

### Schema bump on `Routes::GoalApproach` (header writer + regen)

```cpp
struct GoalApproach {
    uint8_t goal_node;             // NEW — id of the goal node this entry describes
    uint8_t entry_node;
    float   approach_heading_deg;
    const BlindMove* blind_moves;
    uint8_t blind_move_count;
};
```

`tools/path_planner/header_writer.py` emits the new field; golden fixture is regenerated. `tools/path_planner/tests/test_header_writer.py` has its golden compare regenerated as part of the same commit.

### `WRO2026/FieldMap.h` (header-only, namespace, no state)

```cpp
namespace FieldMap
{
    /// @brief Number of nodes in the field graph (re-exported from Routes::).
    constexpr uint8_t N_NODES = Routes::N_NODES;

    /// @brief Sentinel for "no path" / "node not found" / "not a goal".
    constexpr uint8_t NODE_NONE = 0xFF;

    /// @brief Read the next-hop entry from the precomputed table.
    inline uint8_t getNextHop(uint8_t from, uint8_t goal);

    /// @brief Read the directed edge metadata. Caller must have verified that
    ///        `getEdgeType(from, to) != Routes::EdgeType::NONE`.
    inline const Routes::EdgeInfo& getEdge(uint8_t from, uint8_t to);

    /// @brief Read the directed edge type. Returns NONE for non-edges.
    inline Routes::EdgeType getEdgeType(uint8_t from, uint8_t to);

    /// @brief Read the node descriptor by id.
    inline const Routes::NodeInfo& getNode(uint8_t id);

    /// @brief Convenience: just the type field.
    inline Routes::NodeType getNodeType(uint8_t id);

    /// @brief Find the GoalApproach entry whose `goal_node == id`. Returns nullptr if none.
    /// @note Linear scan over `Routes::GOAL_APPROACH[N_GOALS]`. N_GOALS small.
    inline const Routes::GoalApproach* getGoalApproach(uint8_t goalId);

    /// @brief Resolve a node name to its id. Returns NODE_NONE on miss.
    /// @note Linear scan over `Routes::NODES[*].name`. N_NODES small.
    inline uint8_t findNodeByName(const char* name);
}
```

All implementations are inline `return Routes::...[...];` reads. Zero state.

### `WRO2026/Localization.h` (singleton + .cpp impl)

```cpp
class LocalizationClass
{
public:
    bool begin();   ///< currentNode = Routes::START_NODE; _zeroed = false; _yawOffsetDeg = 0.

    /// @brief Capture IMU yaw NOW and compute world-frame offset.
    ///        World heading = normalize(IMU_yaw - yawOffsetDeg).
    /// @return false if MiniR4.Motion is not ready (defensive — should not happen
    ///         once MiniR4.begin() returned).
    bool zeroHeadingFromCurrent();

    bool isZeroed() const;                ///< true after zeroHeadingFromCurrent().
    uint8_t getCurrentNode() const;       ///< single source of truth.
    float getHeadingDeg() const;          ///< world frame, [0, 360). Reads IMU live.

    /// @brief Convert a world-frame heading to the IMU-frame yaw target the
    ///        Drive layer expects. result = normalize(worldDeg + _yawOffsetDeg).
    ///        Returned as int16_t after rounding (TurnGyro takes int16_t).
    int16_t toImuYaw(float worldHeadingDeg) const;

    /// @brief Edge-arrival hook called by PathTracker after a LINE edge.
    ///        Source-of-truth update of currentNode.
    void onIntersection(uint8_t advanceTo);

    /// @brief Edge-arrival hook called by PathTracker after PROJECTION or BLIND
    ///        edges (no physical re-zero, just dead-reckoned advance).
    void onDeadReckonedAdvance(uint8_t advanceTo);

private:
    static float _normalize(float deg);   ///< wrap to [0, 360).

    uint8_t _currentNode = 0;
    float   _yawOffsetDeg = 0.0f;
    bool    _zeroed = false;
};

extern LocalizationClass Localization;
```

### `WRO2026/PathTracker.h` (singleton + .cpp impl)

```cpp
class PathTrackerClass;

class [[nodiscard]] PathTrackerGoToBuilder
{
public:
    PathTrackerGoToBuilder(PathTrackerClass& owner, uint8_t goalId);
    PathTrackerGoToBuilder(const PathTrackerGoToBuilder&) = delete;
    PathTrackerGoToBuilder& operator=(const PathTrackerGoToBuilder&) = delete;
    PathTrackerGoToBuilder(PathTrackerGoToBuilder&&) = delete;
    PathTrackerGoToBuilder& operator=(PathTrackerGoToBuilder&&) = delete;
    ~PathTrackerGoToBuilder();

    PathTrackerGoToBuilder& async();    ///< default = blocking; chain to opt out.

private:
    PathTrackerClass& _owner;
    uint8_t _goalId;
    bool _blocking = true;
    bool _executed = false;
};

class PathTrackerClass
{
public:
    enum class Fault : uint8_t
    {
        NONE = 0,
        NOT_ZEROED   = 1,   ///< goTo before zeroHeadingFromCurrent().
        UNREACHABLE  = 2,   ///< NEXT_HOP[from][goal] == 0xFF.
        DRIVE_FAULT  = 3,   ///< Drive.hasFault() seen during dispatch.
        BAD_NODE     = 4,   ///< goalId out of range / name not found.
    };

    struct FaultInfo
    {
        Fault   kind = Fault::NONE;
        uint8_t fromNode = 0xFF;
        uint8_t goalNode = 0xFF;
    };

    bool begin();
    void update();   ///< cooperative tick — non-blocking.
    void setTickCallback(void (*cb)());
    bool isIdle() const;     ///< true iff queue empty and FSM in IDLE.

    [[nodiscard]] PathTrackerGoToBuilder goTo(uint8_t goalId);
    [[nodiscard]] PathTrackerGoToBuilder goTo(const char* goalName);

    bool hasFault() const;
    FaultInfo getFault() const;
    void clearFault();   ///< clears _fault; does not restart motion.

    /// Called by PathTrackerGoToBuilder::~ — public for the same reason
    /// LineFollower._applyEnableBuilder is public (free-standing builder class).
    void _applyGoToBuilder(uint8_t goalId, bool blocking);

private:
    enum class State : uint8_t
    {
        IDLE,
        DEQUEUE,
        PLAN_NEXT_HOP,
        TURNING,
        LINE_FOLLOW,
        PROJECTION_FOLLOW,
        BLIND_FORWARD,
        GOAL_BLIND_DISPATCH,
        GOAL_BLIND_AWAIT,
        FAULT
    };

    struct GoalEntry { uint8_t goalId; };

    void _drainBlocking();
    void _enterFault(Fault f, uint8_t from, uint8_t to);
    void _dispatchEdge(uint8_t fromNode, uint8_t nextNode);
    void _dispatchBlindStep(const Routes::BlindMove& m);

    GoalEntry _queue[Config::PATH_QUEUE_CAPACITY];
    uint8_t   _qHead  = 0;
    uint8_t   _qTail  = 0;
    uint8_t   _qCount = 0;

    State    _state         = State::IDLE;
    uint8_t  _currentGoal   = FieldMap::NODE_NONE;
    uint8_t  _nextHop       = FieldMap::NODE_NONE;   // pending next-hop node id during TURNING/LINE_FOLLOW/etc.
    Routes::EdgeType _activeEdgeType = Routes::EdgeType::NONE;
    uint8_t  _blindIndex    = 0;
    const Routes::GoalApproach* _activeGoalApproach = nullptr;

    FaultInfo _fault;
    void (*_tickCallback)() = nullptr;
    bool _tickCallbackWarningEmitted = false;
};

extern PathTrackerClass PathTracker;
```

### `WRO2026/LineFollower` extension (new public surface only)

Existing `LineFollowerEnableBuilder` — **no** new copy/move semantics, **no** rename. One added setter:

```cpp
/// @brief Stop after the chassis has traveled @p cm along the line.
///        Implies setEnabled(true) — has no effect on a setEnabled(false) transition.
///        On reach, the controller calls itself the equivalent of
///        setEnabled(false).setBrake(true), then exposes a one-shot
///        LineFollowerClass::isDistanceReached() that PathTracker polls.
LineFollowerEnableBuilder& setDistanceTarget(float cm);
```

Existing `LineFollowerClass` — three new private members and one new public getter:

```cpp
public:
    /// @brief Read-and-clear: true exactly once after the most recent distance gate trip.
    bool isDistanceReached();

private:
    bool    _distanceTargetActive = false;
    float   _distanceTargetCm     = 0.0f;
    int32_t _distanceStartEncL    = 0;
    int32_t _distanceStartEncR    = 0;
    bool    _distanceReached      = false;
```

Inside `update()` after the existing PID branch (and only when `_distanceTargetActive`): compute traveled cm via `cmFromEncDeg(((|encL-startL|) + (|encR-startR|)) / 2)` and on `>= _distanceTargetCm` perform an inline `setEnabled(false)` equivalent (the `_applyEnableBuilder(false, true, …)` private path with brake on, preset preserved). Set `_distanceReached = true`. Cleared by `isDistanceReached()` consumer.

`Config.h` provides the encoder→cm helper threshold (we already have `WROMath::cmToEncDeg`; the inverse can be a one-line `static inline encDegToCm(int32_t deg)` helper inside LineFollower.cpp's anonymous namespace, or extracted to WROMath if a third caller appears).

### `WRO2026/Config.h` additions

```cpp
namespace Config
{
    // ---- PathTracker ----
    /// @brief Maximum number of pending goTo() entries the queue can hold.
    constexpr uint8_t PATH_QUEUE_CAPACITY = 8;

    /// @brief Per-goal worst-case ceiling used by PathTracker._drainBlocking
    ///        to bound its synchronous spin (mirrors DRIVE_DRAIN_TIMEOUT_MS_PER_CMD).
    constexpr uint32_t PATH_DRAIN_TIMEOUT_MS_PER_GOAL = 60000;

    /// @brief Tolerance below which PathTracker skips a turnToYaw dispatch.
    ///        Avoids a tiny twitch at every edge join when consecutive edges
    ///        are colinear (e.g. mid-segment projection node arrival).
    constexpr float NAV_TURN_TOLERANCE_DEG = 0.5f;
}
```

### `WRO2026.ino` (replacement of `DEMO_MODE` with `NAV_DEMO_MODE`)

```cpp
// #define NAV_DEMO_MODE   // Uncomment to enable BTN_UP nav demo on boot.

#include "FieldMap.h"        // header-only, before Localization
#include "Localization.h"
#include "PathTracker.h"
// (existing includes preserved)

void cooperativeTick()
{
    Drive.update();
    LineFollower.update();
    Intersection.update();
    Manipulator.update();
    PathTracker.update();    // NEW — between Manipulator and EventTrace
    EventTrace.update();
}

void setup()
{
    // ... existing MiniR4.begin / Serial / setBattCell ...

    const bool driveOk = Drive.begin();
    const bool lineOk  = LineFollower.begin();
    const bool intOk   = Intersection.begin();
    const bool manipOk = Manipulator.begin();
    const bool locOk   = Localization.begin();
    const bool pathOk  = PathTracker.begin();
    const bool traceOk = EventTrace.begin();

    Drive.setTickCallback(cooperativeTick);
    PathTracker.setTickCallback(cooperativeTick);

    _setupFailed = !(driveOk && lineOk && intOk && manipOk && locOk && pathOk && traceOk);

    // ... existing fail/ok message + EventTrace push ...
}

#ifdef NAV_DEMO_MODE
namespace { bool _navArmed = false; bool _btnPrev = false; }

void loop()
{
    cooperativeTick();
    if (_setupFailed) return;

    const bool btnNow = MiniR4.BTN_UP.getState();
    if (btnNow && !_btnPrev && !_navArmed) {
        _navArmed = true;
        const bool zeroed = Localization.zeroHeadingFromCurrent();
        if (!zeroed) {
            EventTrace.pushEvent("NAV ZERO FAIL");
            return;
        }
        EventTrace.pushEvent("NAV ZERO");
        PathTracker.goTo("C");   // sync-blocks until C reached
        PathTracker.goTo("G");   // sync-blocks until G reached + blind moves done
        EventTrace.pushEvent("NAV DONE");
    }
    _btnPrev = btnNow;
}
#else
void loop() { cooperativeTick(); }
#endif
```

### Python simulator data model

```python
# tools/path_planner/sim/chassis.py
@dataclass
class VirtualChassis:
    x_cm: float
    y_cm: float
    heading_deg: float    # [0, 360)

    def turn_to_heading(self, target_deg: float) -> None: ...
    def turn_relative(self, deg: float) -> None: ...
    def follow_line_to_intersection(self, edge: dict) -> None: ...   # advances by edge.distance_cm
    def follow_line_distance(self, cm: float) -> None: ...
    def forward(self, cm: float) -> None: ...
```

```python
# tools/path_planner/sim/runtime.py
class PathTrackerSim:
    def __init__(self, field_data, graph, next_hop, chassis: VirtualChassis): ...
    def go_to(self, goal_name: str) -> list[Primitive]:
        """Returns the dispatched primitive sequence; mutates chassis in place.
        Raises RuntimeError on unreachable; matches PathTracker.Fault::UNREACHABLE."""
```

## API Contracts

### PR2 module → Drive call boundary

```
PathTracker -> Drive.turnToYaw(int16_t imu_yaw).async().setMode(...)?    (default mode=GYRO turn family)
PathTracker -> Drive.driveDist(float cm).async()                        (BLIND_E and FORWARD blind)
PathTracker -> Drive.stop(true)                                         (FAULT entry)
PathTracker -> Drive.isIdle()                                           (TURNING/BLIND_FORWARD await)
PathTracker -> Drive.hasFault() / Drive.clearFault()                    (drive-fault propagation)
```

### PR2 module → LineFollower call boundary

```
PathTracker -> LineFollower.setEnabled(true)                             (LINE_FOLLOW entry)
PathTracker -> LineFollower.setEnabled(true).setDistanceTarget(cm)       (PROJECTION_FOLLOW entry)
PathTracker -> LineFollower.setEnabled(false)                            (LINE_FOLLOW exit on intersection)
PathTracker -> LineFollower.isDistanceReached()                          (PROJECTION_FOLLOW poll)
```

All builder calls happen in PathTracker.update() — they apply state synchronously inside `_applyEnableBuilder` (no spin), so they don't violate the no-sync-from-update rule.

### PR2 module → Intersection call boundary

```
PathTracker -> Intersection.isAtIntersection()                           (LINE_FOLLOW poll, read-and-clear)
```

EventTrace's existing `getEdgeCount()` use is unchanged.

## Error Handling

- **goTo before zero (`NOT_ZEROED`):** builder destructor checks `Localization.isZeroed()` before enqueue. On false: latches `_fault`, pushes `NAV FAULT NZ`, prints to Serial, returns without enqueue. Subsequent `clearFault()` re-arms but does not retroactively run the rejected goTo.
- **Unreachable goal (`UNREACHABLE`):** detected at `PLAN_NEXT_HOP` when `Routes::NEXT_HOP[from][goal] == 0xFF`. Calls `Drive.stop(true)`, drains the goal queue, latches `_fault`, pushes `NAV FAULT UNR`.
- **Bad node id (`BAD_NODE`):** goalId >= `Routes::N_NODES`, or `findNodeByName` returns `NODE_NONE`. Builder rejects at enqueue time.
- **Drive fault (`DRIVE_FAULT`):** every state-tick that polls Drive also checks `Drive.hasFault()`. If true, calls `Drive.stop(true)` (defensive — Drive may already be stopped), latches PathTracker `_fault.kind = DRIVE_FAULT`, pushes `NAV FAULT DRV`. Caller flow: `PathTracker.clearFault(); Drive.clearFault();` to re-arm.
- **Drain timeout:** PathTracker._drainBlocking has a budget of `(_qCount+1) * Config::PATH_DRAIN_TIMEOUT_MS_PER_GOAL`. On expiry, calls `Drive.stop(true)`, drains the queue, latches `_fault.kind = DRIVE_FAULT`, surfaces.
- **Re-entry:** `_drainBlocking` calls the registered cooperative tick callback (which includes `PathTracker.update()`). `update()` cannot enter `_drainBlocking` because the only path that does is the goTo builder destructor, and PathTracker.update() never constructs a goTo builder. Documented as a `@warning` in PathTracker.h.

## Testing Strategy

**Levels:** Unit + Property (Python sim) + Smoke (C++ compile audit).

| ID | Test Case | Type | Expected Behavior |
|----|----|----|----|
| TC1 | `PathTrackerSim.go_to("C")` from start (currentNode=A, heading=0) | Unit | Primitive sequence equals `compile_path(walk_next_hop(A, C), 0, graph, fd)`; chassis lands within 0.1 cm of C's (60, 60) |
| TC2 | Chained `go_to("C")` then `go_to("G")` mutates chassis through both | Unit | Final chassis at G's `(x, y)` after blind moves; full sequence equals concat of two compile_path calls |
| TC3 | `go_to("A")` when `currentNode == "A"` | Unit | Empty primitive list; chassis unchanged; no fault |
| TC4 | `go_to("G")` when no path exists (test fixture with G fully walled off) | Unit | Raises `RuntimeError` containing "unreachable", "from=...", "to=G" |
| TC5 | `go_to("G")` exercises the goal blind sequence | Unit | Last 2 primitives are `(turn_relative, 0)` skipped (epsilon) and `(forward, 30)` after blind edge; chassis y unchanged, x advanced by full 30 (skipping zero turn) |
| TC6 | 50 random reachable from→goal pairs across the toy graph | Property | sim primitive list == compile_path output for the walked NEXT_HOP path; chassis arrival within 0.1 cm of goal node coords |
| TC7 | `findNodeByName("Z")` (nonexistent) | Unit | Returns `NODE_NONE` (Python: `None`) |
| TC8 | Schema-bumped golden fixture matches re-emitted `routes_toy.h` | Golden | Byte-equal after regen; new `goal_node` field present in every GOAL_APPROACH entry |
| TC9 | C++ compile audit | Smoke | Either arduino-cli reports zero errors, or `docs/phase2-pr2-compile-audit.md` exists with explicit per-file review log |
| TC10 | `Localization.toImuYaw(0.0)` after `zeroHeadingFromCurrent()` returned at IMU=30° | Unit (Python mirror) | Returns `30` (rounded int16) |
| TC11 | `Localization` mirror in Python: `getHeadingDeg()` returns world-frame value tracking simulated IMU updates | Unit (Python mirror) | All-pairs round-trip across `[-720°, +720°]` IMU range |

**Test data:** all tests run against the toy graph fixture (committed in PR1). Property tests generate random goal sequences in-memory; the unreachable test is a custom `field_data`-shaped namespace with an obstacle that fully encloses G.

**Run command:** `uv run pytest tools/path_planner/tests/ -v`

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Branch + Config additions | — | low | `WRO2026/Config.h` | Branch `phase2/path-tracker-runtime` already opened off `main`. Add `Config::PATH_QUEUE_CAPACITY`, `Config::PATH_DRAIN_TIMEOUT_MS_PER_GOAL`, `Config::NAV_TURN_TOLERANCE_DEG`. Doxygen each per the project rule. Satisfies pre-conditions for AC11. |
| T2 | header_writer schema bump + regen | T1 | med | `tools/path_planner/header_writer.py`, `tools/path_planner/tests/golden/routes_toy.h`, `WRO2026/routes.h`, `WRO2026/routes.cpp` | Add `uint8_t goal_node;` as the FIRST field of `GoalApproach` (so the existing `entry_node` keeps its semantic meaning). Update emit logic to write `goal_node` initialiser per goal. Re-run `uv run python tools/path_planner/build_routes.py` to regenerate. Update golden fixture by re-running and committing. Satisfies AC1 partial. |
| T3 | FieldMap.h header-only namespace | T2 | low | `WRO2026/FieldMap.h` | Inline accessors over `Routes::`. `findNodeByName` uses `strcmp`. `getGoalApproach` linear-scans for `goal_node == id`. No .cpp. |
| T4 | Localization.h + .cpp | T1, T3 | med | `WRO2026/Localization.h`, `WRO2026/Localization.cpp` | begin() seeds `_currentNode = Routes::START_NODE`, `_zeroed = false`. zeroHeadingFromCurrent reads `MiniR4.Motion.getEuler(Yaw)`, computes `_yawOffsetDeg = imu_yaw - Routes::START_HEADING_DEG`, sets `_zeroed = true`. getHeadingDeg() reads IMU live and applies offset. toImuYaw rounds via `int16_t`. onIntersection / onDeadReckonedAdvance just write `_currentNode`. extern instance in .cpp. |
| T5 | LineFollower distance gate extension | T1 | high | `WRO2026/LineFollower.h`, `WRO2026/LineFollower.cpp` | Add `setDistanceTarget(cm)` to `LineFollowerEnableBuilder`. Add three new `_distance*` private members + `_distanceReached` one-shot + `isDistanceReached()` read-and-clear. Patch `_applyEnableBuilder` to capture `_distanceStartEncL/R = Drive.getEncL/R()` when target is set. Patch `update()` to evaluate distance gate AFTER the existing PID branch and self-disable on hit (active brake by default). Existing builder defaults preserved. No changes to existing tests. |
| T6 | PathTracker.h + builder + .cpp FSM | T2, T3, T4, T5 | high | `WRO2026/PathTracker.h`, `WRO2026/PathTracker.cpp` | Implement the FSM, queue, fluent goTo builder, drain spin (with cooperative tick callback), fault state, all per the Decisions section. PathTracker.update() must NEVER issue sync verbs. Extern instance. |
| T7 | EventTrace NAV events | T6 | low | `WRO2026/EventTrace.h`, `WRO2026/EventTrace.cpp` | No new public methods; just three new `pushEvent` callsites surfaced via PathTracker / Localization (the actual `pushEvent` is already public on EventTrace). Changes here are limited to any per-event formatting helpers if needed; if every NAV event is a literal string, this task is a doc-only change to Trace's docblock. |
| T8 | WRO2026.ino integration | T6, T7 | med | `WRO2026/WRO2026.ino` | Replace `DEMO_MODE` block with `NAV_DEMO_MODE`. Add Localization + PathTracker to setup() begin chain and to cooperativeTick(). Register PathTracker tick callback. BTN_UP rising-edge zeros heading, runs hardcoded `goTo("C"); goTo("G");`. Motion gate via `_setupFailed`. |
| T9 | Python simulator package | T1 | med | `tools/path_planner/sim/__init__.py`, `tools/path_planner/sim/chassis.py`, `tools/path_planner/sim/runtime.py` | VirtualChassis + PathTrackerSim. Mirror compile_path's logic but emit primitives via the chassis `apply_*` methods so primitive sequence and chassis state evolve together. Imports field_data / graph_builder / planner / compiler directly. |
| T10 | Python simulator tests | T9 | med | `tools/path_planner/tests/test_runtime_sim.py` | TC1–TC7 + TC10–TC11. Property test (TC6) uses `random.Random(seed=0xWRO2026)` for reproducibility. Unreachable test (TC4) builds a custom field_data with G's only-entry segment fully blocked by an inflated obstacle. |
| T11 | PHASE2_DEFERRED.md + spec reconciliation | T10 | low | `docs/PHASE2_DEFERRED.md`, optional one-line note in `docs/PATH_PLANNER_SPEC.md` | Move "End-to-end pure-Python simulator" out of the deferred status table into a short "Phase 2.5 graduates" preamble. Keep wall_zero / calibration nodes / per-scenario tables / Serial console as deferred. One-line note in PATH_PLANNER_SPEC.md §5.1 noting heading is float deg, not 4-way enum. |
| T12 | Final verify | T8, T11 | low | (compile/log only) | Run `uv run pytest tools/path_planner/tests/ -v`. Run `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` if installed; else write `docs/phase2-pr2-compile-audit.md` per AC6. Squash-merge-ready commit log. |

T1 first; everything depends on it. T2, T3, T4, T5, T9 can in principle run in parallel after T1, but I'm executing sequentially as one agent so order is T1 → T2 → T3 → T4 → T5 → T6 → T7 → T8 → T9 → T10 → T11 → T12.

## Notes for Implementer

- **PathTracker.update() is a non-blocking FSM tick.** It MUST NOT issue any sync-blocking Drive or LineFollower verb. Every Drive call is `.async()`. LineFollower builders apply their state synchronously inside `_applyEnableBuilder` — that's fine; what's forbidden is calling `Drive.driveDist(...)` (sync default), `Drive.turnToYaw(...)` (sync default), `Drive.waitUntilIdle()`, etc.
- **The goTo() builder's destructor is the only sync site in PathTracker.** It calls `_drainBlocking()` which loops `_tickCallback()` (or falls back to ticking `update()` alone with a one-time Serial warning). The cooperative tick re-enters `update()` repeatedly during the spin — this is expected and safe because update() doesn't drain anything itself.
- **goTo to currentNode is a no-op:** in PLAN_NEXT_HOP, if `_currentGoal == Localization.getCurrentNode()`, skip directly to GOAL_BLIND_DISPATCH if it's a goal, else IDLE/DEQUEUE. Don't issue a turnToYaw — there's nothing to turn to.
- **Heading near-zero turns:** if the absolute delta after `normalize_turn(target - current)` is `< Config::NAV_TURN_TOLERANCE_DEG`, skip the turn dispatch entirely and transition straight from PLAN_NEXT_HOP to the appropriate motion state. Avoids a sub-degree twitch on every projection-node hand-off.
- **LineFollower's `_distanceTargetActive` is one-shot.** When the gate trips, the controller transitions itself out of the enable-with-target state. PathTracker reads `isDistanceReached()` (read-and-clear) to confirm. A second `setEnabled(true).setDistanceTarget(cm)` on a fresh edge re-arms cleanly.
- **BTN_UP debounce:** the existing _btnPrev rising-edge guard in WRO2026.ino is sufficient. The cooperative tick rate is high enough that a single-tick true sample is real enough.
- **`_zeroed = false` after begin() is intentional.** Powering on without zeroing is a state Diego's flow specifically wants to fault loud rather than silently use a default offset of zero — see Decisions section "goTo before zero is a fault."
- **TurnGyro async firmware uncertainty:** if hardware bench testing reveals the lower-MCU firmware silently blocks despite `async=1` on the wire, the cooperative tick will freeze for the duration of each turn (~1–2 s). For PR2 this is acceptable (chassis stationary during turns). If it becomes a problem in PR3, the fallback is to issue turns from `goTo()`'s drain phase synchronously — a 5-line change in PathTracker.cpp's TURNING state handler.
- **N_GOALS edge case in FieldMap::getGoalApproach:** if `Routes::N_GOALS == 0` (e.g. a future test fixture with no goals), the loop returns `nullptr` immediately. Defensive — the toy fixture has N_GOALS = 1 so this path is hit on miss only.
- **Schema bump on GoalApproach:** ordering of fields matters for the existing test_header_writer golden compare. Put `goal_node` first to keep `entry_node`'s position changeable later without further churn. The auto-generated comment in routes.cpp should say `// goal G (id 5)` for readability — header_writer already emits that comment.
- **Python simulator's heading propagation must match `compile_path` byte-for-byte.** A tiny float epsilon mismatch (e.g. `math.fmod` vs `% 360.0`) will silently break the property test on edge cases. Re-use `compile_path`'s `normalize_turn` helper directly rather than re-implementing.
- **Rollback plan:** PR 2 is mostly additive (new files in `WRO2026/`, `tools/path_planner/sim/`). The risky deltas are the `LineFollower` extension and the `WRO2026.ino` DEMO_MODE replacement. Rollback = revert the merge commit; sketch falls back to Phase 1 behavior cleanly.
- **PR 3 hand-off contract:** PR 3 will replace `field_data.py` content (real 2026 field) and add a Serial-console mission entrypoint that calls into `PathTracker.goTo(...)`. The PathTracker public API surface should not need to change.
