# Phase 2 Deferred Features

PR 1 of Phase 2 (`phase2/path-planner-python`) shipped a working offline build pipeline: field data
in `tools/path_planner/field_data.py` is validated, turned into a weighted graph, run through
Dijkstra all-pairs, compiled to a primitive sequence with correct heading propagation, and emitted
as `WRO2026/routes.h` + `WRO2026/routes.cpp`. A matplotlib visualization (`map.png`) and a
property-based `pytest` suite are included. The R4 runtime that consumes `routes.h` is PR 2;
the real 2026 field encoding and Serial-console mission entrypoint are PR 3. Several features
described in `docs/PATH_PLANNER_SPEC.md` were intentionally left out of PR 1 to keep the diff
small and reviewable. This document records each deferred feature so that a future contributor
(or a future session of this same Claude instance) can revive the work without re-reading the
entire conversation history.

---

## Status table

| Feature | Spec § | Status | When likely |
|---|---|---|---|
| Calibration nodes + blind-cost tiers + constrained Dijkstra | §8.5 | Deferred to Phase 2.5 | After PR 3 ships real-field nav and accumulated blind-walk drift is observed |
| Per-scenario NEXT_HOP tables (transparent self-target) | §7.4 | Deferred to Phase 2.5 | After PR 3 encodes multiple goals on the real field |
| `wall_zero` primitive on the R4 | §5.3 | Deferred to Phase 2.5 | After mechanical hard-bumper or limit-switch work is done |
| End-to-end pure-Python simulator | §6 step 9 | Deferred to Phase 2.5 | After first real-field plan misbehaves and needs offline replay |
| Serial-console mission entrypoint | PR 3 scope | Deferred to PR 3 | After PR 2's PathTracker exists |

---

## Calibration nodes + blind-cost tiers + constrained Dijkstra

### What it is

Section 8.5 of the spec defines a mechanism for actively correcting accumulated odometry error.
Certain nodes are flagged as calibration points; when the robot traverses one it executes a
`wall_zero` primitive to re-anchor its position and heading. The spec states:
> 「盲走的不確定性與累積距離成正比。剛校正完誤差小，盲走風險低，cost 應低；久沒校正誤差大，
> 盲走風險高，cost 應高。」

The cost mechanism uses five 5 cm tiers:

```
Cumulative blind distance  0– 5 cm  →  cost × 1.0
                           5–10 cm  →  cost × 2.0
                          10–15 cm  →  cost × 3.0
                          15–20 cm  →  cost × 4.0
                          20+  cm   →  cost × 5.0  (baseline)
```

Represented in `field_data.py` as:

```python
BLIND_COST_TIERS = [
    (0,  5,  1.0),
    (5,  10, 2.0),
    (10, 15, 3.0),
    (15, 20, 4.0),
    (20, float("inf"), 5.0),
]
```

Because the cost of a blind edge depends on how much blind distance has accumulated since the last
calibration, standard Dijkstra (which assumes path-independent edge costs) does not apply. The spec
calls for an augmented state `(node, blind_bucket)`, expanding the state space from N to N × 5.
The R4's `NEXT_HOP` table becomes three-dimensional: `[from][blind_bucket][to]`, and the R4 must
maintain a `blind_accumulated_cm` counter at runtime.

### Why deferred

The toy graph in PR 1 has a single short blind segment. The tier mechanism would be inert on it,
and the augmented-state Dijkstra adds non-trivial search complexity (N × 5 states, custom priority
queue, bucket discretisation logic) for zero observable benefit on a toy fixture. The 3-D
`NEXT_HOP` table also complicates `routes.h` before PR 2's PathTracker even exists.

### What unlocks it

Real 2026 field is encoded in `field_data.py` (PR 3); multiple blind segments exist in the
competition route; drift is observed on hardware to accumulate enough to cause mis-localisation on
a multi-blind task; and the chassis reliably supports `wall_zero` (see that entry below).

### Estimated effort

Medium. `graph_builder.py` must attach calibration metadata to nodes. `planner.py` must implement
a custom priority-queue Dijkstra over `(node, blind_bucket)` state, replacing the `networkx`
single-source call for affected runs. `header_writer.py` must emit a 3-D NEXT_HOP array.
`tests/test_planner.py` needs augmented-state test cases. The R4 side (PR 2 / PR 3 scope) needs a
`blind_accumulated_cm` counter and updated lookup logic. Total: ~200–300 lines of Python + C++
changes, spread across four modules.

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

## `wall_zero` primitive on the R4

### What it is

Section 5.3 lists `wall_zero(direction)` in the primitive table alongside `follow_line_distance`
and `turn_to_heading`. Its role is described in §5.4:
> 「wall_zero：對應軸的位置 + heading」

The primitive drives the robot into a known wall surface until contact is detected (encoder stall
or a limit switch triggers), then backs off a fixed distance. The result is an absolute-reference
reset of position and heading on the contacted axis. Section 8.5.7 notes that the plan compiler
auto-inserts `wall_zero` after every calibration node arrival — no user-authored step is needed.

### Why deferred

At the time of PR 1 the robot has no front bumper and no limit switch. Reliable contact detection
requires a mechanical decision (hard bumper vs. limit switch vs. encoder-stall threshold), and the
back-off distance must be tuned to the specific surface and motor characteristics. Writing the C++
primitive before the mechanical design is finalised would produce dead code that is likely wrong.

### What unlocks it

A physical hard bumper or limit switch is installed on the robot's leading face. Once mechanical
geometry is settled, `wall_zero` is a small Drive primitive on the R4 (push at low speed until
stall or switch fires, then reverse a fixed `back_off_cm`) plus a `Localization::resetAxis` hook.
The calibration-node feature (above) depends on this primitive, so both features likely land in the
same PR.

### Estimated effort

Small on the code side: one new primitive function in `primitives.cpp` (~40 lines of C++) and a
`Localization::resetAxis` method (~20 lines). The hard part is the mechanical work and the
per-surface back-off calibration, which is outside the software scope.

---

## End-to-end pure-Python simulator

### What it is

Section 6, step 9 calls for:
> 「End-to-end 模擬 — 在電腦上模擬執行」

The idea is a pure-Python "fake R4" that accepts the compiled primitive list from `compiler.py`,
advances a virtual chassis state `(x, y, heading_deg)` for each primitive, and returns or prints
the full trace. This lets a developer verify a planned route without uploading to hardware: does
the simulated robot end up at the right coordinates? Does it clear all obstacles?

PR 1 provides `compiler.py` (primitive emission) and the `pytest` suite (plan-correctness checks
on the graph and heading logic), which together cover most of the verification value. A separate
visual simulator mainly adds value as a replay tool for diagnosing hardware failures.

### Why deferred

The compiler's unit tests already validate plan correctness at the primitive level, and the
visualizer's `map.png` gives spatial intuition. A full simulator only pays off when there is a
real multi-step field mission that misbehaves on hardware and needs an offline "what would the
chassis have done?" trace. That situation does not arise until PR 3 is running on the real field.
It is also valuable as an onboarding aid; that, too, is a PR 3-era concern.

### What unlocks it

First instance of a real-field plan executing unexpectedly on hardware, or the first time a new
contributor needs to understand plan execution without robot access. Alternatively, if the test
suite for PR 3's field data grows large enough that hardware iteration becomes the bottleneck,
the simulator pays for itself immediately.

### Estimated effort

Small. The simulator is a self-contained module (~150 lines of Python) that consumes the existing
`CompilerResult` data structure from `compiler.py` and has no new dependencies. Tests are ~50 lines
of fixture-driven assertions against known routes on the toy graph. No changes to `routes.h`,
`planner.py`, or any R4 code.

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

1. Re-read `docs/PATH_PLANNER_SPEC.md` and the subsection above for the specific feature.
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
