# Phase 2 Deferred Features

Phase 2 shipped through PR3 (Mission DSL). The offline Python pipeline, the R4 runtime
(`FieldMap`, `Localization`, `PathTracker`), the pure-Python simulator, and the operator
Mission DSL are all in `main`. This document tracks the work that was scoped out of PR1/PR2/PR3
and remains genuinely deferred — usually because it depends on real 2026 field geometry or
on mechanical hardware that doesn't exist yet.

The detailed design narratives below are preserved so a future contributor can revive each
feature without re-reading the PR conversations. Items shipped during Phase 2 are recorded
in the "Graduates" subsection.

## Graduates (already shipped)

- **End-to-end pure-Python simulator** (spec §6 step 9). `tools/path_planner/sim/` +
  `tools/path_planner/tests/test_runtime_sim.py`. Mirrors PathTracker in pure Python and asserts
  primitive-sequence equality + chassis arrival within 0.1 cm.
- **Operator mission entrypoint** (was "Serial-console" in the spec). Shipped as the **Mission
  DSL** in PR3 (`Mission.h/.cpp` + inline `buildMission()` in `WRO2026.ino`). Verbs:
  `goTo` / `wait` / `print` / `beep` / `led`.
- **NAV FAULT mnemonics** (NZ / UNR / DRV / BAD / DRN). Shipped in `PathTracker::faultMnemonic()`
  and emitted to Serial on every fault transition. EventTrace also pushes a `FAIL PATH` line
  on the OLED.
- **`wall_zero` primitive (§5.3)** — shipped via the **encoder-stall** path (no bumper / limit
  switch). A `"reset"` node kind carries a sparse `RESET_NODES{axis, wall_coord, facing}` table;
  on arrival PathTracker runs `Drive.wallStop` (push until the stop-at-line stall watchdog fires,
  back off) then one atomic `Localization.reanchorAtWall(axis, coord, heading)`. Recorded in
  [ADR-043](./decisions/043-wall-zero-reset-nodes.md). The **hardware-switch variant** (bumper /
  limit switch on the leading face) remains a future option — see §5.3 below for that trigger.

---

## Status table

| Feature | Spec § | Status | When likely |
|---|---|---|---|
| Calibration nodes + blind-cost tiers + constrained Dijkstra | §8.5 | **Partially graduated** — static multiplicative decay shipped via ADR-041; path-dependent augmented-state `(node, blind_bucket)` + runtime counter still deferred | After real-field runs show the static tail-approximation mis-routing in practice |
| Per-scenario NEXT_HOP tables (transparent self-target) | §7.4 | Deferred to Phase 2.5 | After the real 2026 field encodes multiple goals |
| `wall_zero` primitive on the R4 | §5.3 | **Graduated** — shipped via encoder-stall (ADR-043); hardware-switch variant still future | Bumper / limit-switch variant: after mechanical hard-bumper or limit-switch work is done |
| Hardware verification of PathTracker on the toy printout | PR2 scope | Deferred to bench session | Once mechanical chassis geometry is finalised |
| IMU drift compensation across long runs | PR2-new | Deferred | Once the real 2026 field's longer paths expose drift accumulation |
| Replace `field_data.py` with the real 2026 layout | PR3 followup | Deferred | After field is measured via `pick_xy.py` (issue #10 scope 1 shipped the tool) |

---

## Calibration nodes + blind-cost tiers + constrained Dijkstra — PARTIALLY GRADUATED (static approximation shipped; path-dependent form still deferred)

> **Partially graduated.** The **static approximation** shipped via the blind-decay routing
> feature (PR3) — see [ADR-041](./decisions/041-blind-decay-routing.md). The offline planner
> weights blind edges by a convex multiplicative factor
> `weight = distance_cm * (1 + k * (d(u) / d_ref) ** exponent)` where `d(u)` is the
> blind-subgraph distance from tail node `u` to its nearest anchor (any line or reset node),
> computed once at table-build time. One flat `NEXT_HOP[from][to]` table is emitted; no runtime
> counter; no firmware change. This replaces the old flat `COST_BLIND_MULTIPLIER`.
>
> The **path-dependent augmented-state form** — `(node, blind_bucket)` state, 3-D
> `NEXT_HOP[from][blind_bucket][to]`, runtime `blind_accumulated_cm` counter — remains deferred.
> Its upgrade trigger: real-field runs show the static tail-approximation actually mis-routing
> (the directional naivety biting in practice). The design narrative below is preserved for that
> future upgrade path.

### What the static model does not capture

The static decay is keyed on the edge **tail** (a fixed potential field), so it is directionally
naive — per-edge, travelling toward an anchor can cost more than travelling away — and it gives
the first blind hop off an anchor zero penalty. The model discourages deep blind **chains**, not
single bounded hops, which is its stated goal. The section below describes the full path-dependent
form that fixes the directionality.

### What the path-dependent form adds (still deferred)

Section 8.5 of the spec calls for a cost that reflects how much blind distance has accumulated
**since the last calibration node**, not a static node potential. The spec states:
> 「盲走的不確定性與累積距離成正比。剛校正完誤差小，盲走風險低，cost 應低；久沒校正誤差大，
> 盲走風險高，cost 應高。」

The spec's tier representation:

```
Cumulative blind distance  0– 5 cm  →  cost × 1.0
                           5–10 cm  →  cost × 2.0
                          10–15 cm  →  cost × 3.0
                          15–20 cm  →  cost × 4.0
                          20+  cm   →  cost × 5.0  (baseline)
```

Because the cost of a blind edge depends on path history, standard Dijkstra does not apply. The
spec calls for augmented state `(node, blind_bucket)`, expanding the state space from N to N × 5.
The R4's `NEXT_HOP` table becomes three-dimensional: `[from][blind_bucket][to]`, and the R4 must
maintain a `blind_accumulated_cm` counter at runtime. Traversing a calibration (reset) node resets
the bucket to 0.

### What unlocks the path-dependent upgrade

Real-field runs show the static tail-approximation mis-routing — the planner choosing a
deep-blind path over a line-threading route of comparable total length because the directional
naivety of the potential-field model fails to capture the path-history cost. At that point the
augmented-state form is the upgrade.

### Estimated effort (path-dependent form)

Medium. `planner.py` must implement a custom priority-queue Dijkstra over `(node, blind_bucket)`
state, replacing the `networkx` single-source call. `header_writer.py` must emit a 3-D NEXT_HOP
array. `tests/test_planner.py` needs augmented-state test cases. The R4 side needs a
`blind_accumulated_cm` counter and updated lookup logic. Total: ~200–300 lines of Python + C++
changes across four modules.

---

## Per-scenario NEXT_HOP tables (transparent self-target)

### What it is

Section 7.4 explains that a Mosaic tile starts as an obstacle (the robot must not collide with it
while navigating to other goals), but becomes the destination when it is the current target. The
spec's solution is to generate one NEXT_HOP table per goal scenario:
> 「每個 goal 產生一張獨立的 NEXT_HOP 表，那張表裡對應的 my_target 障礙物被忽略。
> R4 端依據當前任務選對應的表。」

`OBSTACLES` entries with `kind = "my_target_<goal>"` are omitted from the obstacle set when
planning for that goal. The R4 calls a `pick_scenario_table(goal_id)` helper (sketched in the spec,
§5.2) to select the right table at runtime.

PR 1's `planner.py` emits a single global NEXT_HOP table, which corresponds to the "baseline"
scenario where all obstacles are solid.

### Why deferred

The toy graph has one goal and zero `my_target_*` obstacles, so a second scenario table would be
identical to the first — the machinery would be there but untestable. Memory cost on the R4 grows
as N × N × G bytes; with G = 1 that cost is zero, making it hard to evaluate the budget impact.
PR 2's PathTracker also does not yet have a `pick_scenario_table` call site to wire into.

### What unlocks it

PR 3 encodes the real field, which will likely have ~4 mosaic-tile goals and ~3 cement-deposit
goals. Once `my_target_*` obstacles exist in `field_data.py` and the R4 has confirmed RAM headroom
for an N × N × G table (spec §7.11 estimates 5 scenarios × 400 bytes = 2 KB on a 32 KB R4), the
feature is straightforward to activate.

### Estimated effort

Small. `planner.py` gains a `goal_id` parameter and an obstacle-filter step before building the
graph per scenario. `header_writer.py` emits a 3-D `NEXT_HOP[goal][from][to]` array and a
`pick_scenario_table` inline that indexes into it. The PathTracker in PR 2 indexes on `goal_id`
when calling `go_to`. Net new code: approximately 50–80 lines of Python + 20 lines of C++.

---

## `wall_zero` primitive on the R4 — GRADUATED (shipped via encoder-stall; hardware-switch variant remains future)

> **Graduated.** The software primitive shipped via the **encoder-stall** path (no bumper / limit
> switch) — see [ADR-043](./decisions/043-wall-zero-reset-nodes.md). A `"reset"` node kind carries
> a sparse `RESET_NODES{axis, wall_coord, facing}` table; on arrival PathTracker runs `Drive.wallStop`
> (push until the stop-at-line stall watchdog fires — stall = success — then back off) then one
> atomic `Localization.reanchorAtWall(axis, coord, heading)`. The **hardware bumper / limit-switch
> variant** below remains the future upgrade trigger; the section is kept for that path's design
> narrative.

### What it is

Section 5.3 lists `wall_zero(direction)` in the primitive table alongside `follow_line_distance`
and `turn_to_heading`. Its role is described in §5.4:
> 「wall_zero：對應軸的位置 + heading」

The primitive drives the robot into a known wall surface until contact is detected (encoder stall
or a limit switch triggers), then backs off a fixed distance. The result is an absolute-reference
reset of position and heading on the contacted axis. Section 8.5.7 notes that the plan compiler
auto-inserts `wall_zero` after every calibration node arrival — no user-authored step is needed.

### What shipped

The encoder-stall variant landed as the wall-zero reset node feature (ADR-043): `Drive.wallStop`
(a continuous, stall-terminated push that reuses the stop-at-line no-progress watchdog with the
stall meaning inverted — a stall is success) plus `Localization.reanchorAtWall(axis, wall_coord,
heading)`, dispatched by PathTracker's `WALL_STOP` → `REANCHOR` arrival interlude. No new hardware
was required.

### What still unlocks the hardware variant

A physical hard bumper or limit switch installed on the robot's leading face. The encoder-stall
detector is the only mechanism shipped; if it proves unreliable on the real field (false stalls,
or no stall against a compliant wall), the bumper / limit-switch contact detector is the upgrade —
it would route through the same `Drive.wallStop` → `reanchorAtWall` interlude, swapping only the
contact-detection source. The optional strict wall-normal check
(`Config::WALL_NORMAL_STRICT_ENABLED`, off by default) is the in-software escalation that flags a
bad post-stop heading in the meantime.

### Estimated effort (hardware variant only)

Small on the code side: a new contact-detection source feeding the existing `wallStop` state
machine (the `Localization.reanchorAtWall` hook already exists). The hard part is the mechanical
work and the per-surface back-off calibration, which is outside the software scope.

---

## Serial-console mission entrypoint

### What it is

The user described the intended UX: key in a sequence of node names (e.g. `A B C D`) over Serial,
and the robot automatically navigates to each in order. This is the human-facing wrapper around
PR 2's `PathTracker::goTo(node_id)` call. It lives on the R4 and belongs to the mission / task
layer (spec §1.1, row "任務層").

This feature is not explicitly named in `PATH_PLANNER_SPEC.md` as a spec section, but it is the
natural capstone of the three-PR arc: PR 1 generates the table, PR 2 makes the robot follow it,
PR 3 gives the user a way to drive it interactively.

### Why deferred

PR 1 is the Python pipeline only — no R4 code at all. PR 2 will land the PathTracker runtime but
will be driven programmatically (a hardcoded sequence in `main.cpp`). The Serial-console entrypoint
is cleanest as its own PR 3 addition: parse incoming Serial tokens, resolve node names via the
`Routes::` namespace, queue `goTo` calls. Doing it in PR 1 or PR 2 would mean writing against an
interface that does not yet exist.

### What unlocks it

PR 2's `PathTracker::goTo` is merged and verified on hardware with the toy graph. Once that
contract is stable, the Serial entrypoint is roughly 80 lines of Arduino C++: read a `\n`-delimited
token stream, look up the node index in `Routes::NODES` by name comparison, call `goTo`. A small
command grammar (e.g. `go A`, `go B C D`, `stop`, `status`) makes it usable without a custom
host-side tool.

### Estimated effort

Small. ~80 lines in `main.cpp` or a new `SerialMission.h`. No changes to `routes.h`, `planner.py`,
or any existing R4 module. The naming lookup could use a short linear scan over `Routes::NODES`
(N ≤ 20 on the toy graph, O(N) is fine at human interaction speed).

---

## How to revive a deferred feature

1. Re-read the archived spec at `docs/archive/PATH_PLANNER_SPEC.md` and the subsection above for the specific feature.
2. If the feature changes `field_data.py`'s schema (e.g. adding `CALIBRATION_NODES` or
   `my_target_*` obstacle kinds), update `validator.py` first and add/update tests in
   `tests/test_field_data.py`.
3. Add or modify the feature's logic in the appropriate module:
   - Schema / data: `field_data.py` + `validator.py`
   - Graph topology: `graph_builder.py`
   - Search / planning: `planner.py`
   - Heading + primitive emission: `compiler.py`
   - C++ output: `header_writer.py`
   - Visualization: `visualizer.py` (if new node/edge types need new colours)
4. Update `WRO2026/routes.h` schema and the golden fixture at
   `tools/path_planner/tests/golden/routes_toy.h`; re-run `build_routes.py` to regenerate and
   commit the new golden file alongside the code change.
5. If the `routes.h` public interface changes (new arrays, changed types), update PR 2's
   PathTracker and Localization accordingly. The R4 side is the consumer; breaking it silently
   is worse than blocking the Python build.
6. Document architectural shifts in `docs/DECISIONS.md` and, if the change affects day-to-day
   contributor workflow, in `CLAUDE.md`.
