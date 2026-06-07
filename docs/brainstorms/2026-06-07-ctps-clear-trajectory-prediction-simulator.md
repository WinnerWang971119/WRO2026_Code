# Brainstorm: CTPS (Clear Trajectory Prediction Simulator)

**Seed (verbatim):**
> # Idea
> - The CTPS is a system that can replace the current sim
> - It creates a playable sim instead of a graph
> - Like the FRC sim but simpler
> - It helps the operator to see is the route of the robot is correct and predict possible problema
>
> # Possible Direction
> To achive the CTPS , we have to establish a R4 like system and at the same time do what the path tracker is doing, and it should sync with the mission (tasks) simotainously. Like when we write task 1 -> 2 -> 3. The sim will also do 1 2 3, which is the whole point of making a sim. Need to discuss whether to create like a jason file to set the task and mission each time, then compile them into task and mission in both the runtime and the sim. The sim is also possible to be reused to use at the LRS(log replay system). Need to check and ddiscuss.

**Context summary:**
- Current sim = pure-Python `PathTrackerSim` (`tools/path_planner/sim/runtime.py`), a faithful FSM mirror of C++ PathTracker that emits a *primitive stream* (not tick-by-tick). `sim/demo.py` prints a text trace + optional matplotlib PNG, still driven by a `--goals A,B` CLI flag.
- Tasks framework (PR #35) **merged to main 2026-06-06**. The `_registry[]` in `Tasks.cpp` (`{name, fn}` rows) is the natural sim contract; EventTrace emits `TASK START/OK/YIELD` boundary events.
- Saved rule + Roadmap P5.8: the sim must load the robot's **actual Mission script** (drop `--goals`) once Tasks ship. P6: sim runs the full Mission end-to-end as the dry-run gate.
- LRS (issue #23) is a separate-but-adjacent system: log encoder+sensor data on robot, replay tick-by-tick on PC. Concerns: tick-rate cost, no SD card on R4 Mini, memory. ADR-010 currently keeps logging "lightweight only"; replay tooling is deferred (P9.5 optional).
- **Hard constraint:** user strongly prefers a C++ fluent DSL over JSON for mission/task authoring (ADR-025, "Strong No" 3.75/10 on the JSON idea). The issue's own "maybe a JSON file" line runs against this documented preference — a key tension to resolve.

**Clarified framing:** CTPS v1 is a **scrubbable 2D timeline player** (play / pause / seek) that animates the robot driving the *actual committed* Mission script over the field map, mirroring the real PathTracker + Tasks FSM so an operator can watch the planned route + task sequence and spot problems before a hardware run. It replaces today's static-trace `sim/demo.py` and kills the `--goals` flag (Roadmap P5.8 / P6). The **mission/task source-of-truth fork** (C++ DSL + codegen vs shared JSON vs robot-emitted trace) is the central open question to pressure-test. The renderer should be architected so the future LRS (issue #23) can feed real *logged* traces into the same player, but LRS itself is out of v1 scope (CTPS first, design viz for reuse). Outcome: hand to `/plan`.

## Perspectives (round 1)

### Critic
1. **"Playable/scrubbable" is a category jump the sim can't make** — `VirtualChassis._advance` applies each primitive's full displacement atomically ("No timing, no encoder model, no IMU model"); `runtime.py` is "not tick-by-tick." A timeline needs a continuous t→pose fn (accel ramps, turn duration). Either build a second physics engine (new parity surface vs `Drive.cpp`) or fake "play" by linear interp between atomic endpoints — an animation that hides the very accel/overshoot behavior the operator wants to predict.
2. **Parity/drift machinery is already fragile and load-bearing; richer model multiplies it** — runtime.py hand-mirrors many `Config.h` constants ("MUST mirror", regex-checked), byte-identical TC12/TC18 streams, hand-duplicated resolvers. A velocity/timing model adds a whole new class of constants to mirror (accel, cruise, brake) with no automated C++ check — exactly the values Phase 3 hardware tuning will churn.
3. **"Sync with tasks" core promise is blocked: Task bodies are opaque C++** — a `TASK` step dispatches `bool TasksClass::<name>(const TaskArgs&)` calling PathTracker/Drive/Buzzer directly. Sim can read registry *names* but not execute *bodies*. ADR-025 admits the mirror requires hand-writing a Python mirror of each task body — Phase 5/6 features without hardware decisions yet.
4. **Scope on a non-critical path during Phase 3 bring-up before regionals** — robot never verified on real hardware; Tasks/manipulator/camera unbuilt. A scrub player implies an event loop/GUI/web stack the repo lacks (only matplotlib; TkAgg already caused headless-CI bugs).
5. **"Predict problems" is unfalsifiable as architected** — sim runs the same NEXT_HOP + resolvers on idealized geometry, no sensor noise/slip/drift/line-loss (`on_intersection` just sets the node). Tautology engine: reachable plan → shows reaching goal by construction. Catches authoring typos only — which Mission already catches at queue time.
6. **LRS coupling pulls unsolved scope into v1 as a constraint** — LRS is itself unsolved (no SD card, tick-rate cost, ADR-010 lightweight-only). Designing CTPS for a tick-by-tick logged trace forces a tick schema for a consumer that may never ship; violates CLAUDE.md "no speculative abstractions."

### Feasibility
**VERDICT: Conditional.** No hard blockers — nothing is impossible on the Python-local, no-host-compiler toolchain.
Must-be-true: (1) add a time/velocity model as a NEW additive layer over the existing primitive stream — `step_through()` already yields per-step pose snapshots, so the atomic FSM is NOT rewritten, just wrapped + interpolated (constant-velocity is enough for v1; T-shape later). (2) resolve the Mission/Tasks source-of-truth fork — today Mission is C++-only (`buildMission()` in the .ino), there is NO Python mirror of Mission/Tasks (parity mirrors PathTracker only); `_registry[]` gives names not bodies. (3) decouple renderer as a generic timestamped-pose+events consumer for LRS reuse.
Solvable: Mission ingestion has 3 implementable options — (1) **hand-mirrored Python Mission/task bodies** following the *exact existing parity pattern* (ADR-025 pre-authorizes this; de-risked default, reuses proven golden-test machinery); (2) **robot-emitted EventTrace trace** the player replays (cleanest, doubles as LRS format, BUT replay-only = no pre-run prediction, which is the issue's purpose); (3) **codegen from the C++ DSL** like routes.h (ADR-025's "Strong No" is against JSON *authoring*, not codegen). Player tech: matplotlib FuncAnimation+Slider (zero new deps, clunky seek) OR a self-contained HTML/Canvas player (native play/seek, better LRS host, new JS surface). WPILib Field2d/Glass confirms the shape is standard but is a *live dashboard*, not a pre-run predictor — so replay-only (opt 2) alone won't satisfy "predict before run"; a route/mission *model* (opt 1 or 3) is needed.

### Thinker (branches)
1. **Timeline scrubber driven by EventTrace boundary events** — CTPS consumes the same telemetry vocabulary EventTrace already emits (TASK START/OK/YIELD, NAV FAULT, HB heartbeat). CTPS and LRS become the *same player* with two input adapters (sim-generated vs robot-logged); the JSON-vs-DSL question dissolves — the format is the event vocabulary.
2. **Give VirtualChassis a clock so "playable" means real-time** — each primitive gets a wall-clock cost from the T-shape profile + turn-rate model, so play/pause/seek map `t`→"which primitive, how far through." Unlocks timing prediction free ("you'll blow the 2-min budget at task 4"). Cost: optional timing parity surface (visual layer only).
3. **CTPS as a golden-snapshot diff viewer, not a forward simulator** — two-trace overlay: scrub current mission with a ghost of the *previous committed mission* (or last hardware run via LRS), divergences in red. Steals AdvantageScope "compare two logs" + CI golden mechanic. "Predict problems" → "this edit rerouted leg 3 through the obstacle corridor."
4. **Mission authoring *inside* the player — scrub-to-edit** — drag task markers / click a node to drop `goTo(node)` / reorder legs, watch it re-drive; the player *emits* the `buildMission()` DSL (honors ADR-025: player is a DSL generator, not a JSON fork). Fuses with field editor #10. v1 = one-way emit (player→DSL), hand edits authoritative.
5. **A "problem oracle" layer that annotates the timeline with predicted faults** — pluggable pure-function checkers over the primitive+pose stream: collision (footprint ∩ OBSTACLES w/ C-space inflation), overshoot (LINE_OVERSHOOT overruns next node), unreachable, heading-drift accumulation, reverse-seek-without-rear-board silent FRONT fallback. Render as colored timeline pins; ALSO run headless in CI as a regression gate. This is where CTPS earns its keep over the static sim.
6. **Browser-native player as a self-contained HTML+SVG artifact** *(speculative)* — pipeline emits one `ctps.html` (field SVG + embedded pose/event JSON + ~200 lines vanilla JS transport bar). Double-click to run, zero deps, trivially shareable, trivially loads an LRS track later. Downside: new JS surface; widens the deliberately-narrow toolchain.
7. **Time-travel debugger framing: step the FSM, not the clock** *(speculative)* — transport = step-fwd/back over PathTracker *states*, inspector shows current_node, resolved Direction, chosen BlindMode, why stop-at-line picked seek-vs-dead-reckon, pending NEXT_HOP. Scrub *decisions*, not centimeters. Developer audience; no timing model needed.
8. **One unified "field replay engine," three thin frontends (convergence play)** — build a shared `TrackPlayer` over `(field_geometry, [pose_or_event @ t])` once; CTPS/LRS/field-editor/parity-suite become thin adapters. Parity generalizes from primitive-stream equality to pose-track equality. Makes "the sim could be reused at the LRS" structural, not aspirational. v1 = core + CTPS adapter.

## Ingestion mechanisms (Thinker round 2 — "how does the sim read the mission/tasks")
Fares key: no-local-compiler · ADR-025(no-JSON-authoring) · single-source-vs-drift · works-without-run

**A. Keep C++ as source, derive Python**
1. **CI host-build emits self-description artifact** ⭐ — add a "describe" target to the existing `WRO2026/test/` cmake harness; recording mocks capture `buildMission()`'s real executed `goTo/task` + in-task command sequence → dump JSON; CTPS reads it. needs-CI · consistent (derived, like routes.h) · TRUE single-source, highest fidelity · works-without-run. Catch: locally stale until CI run (same as routes.h freshness gate).
2. **libclang AST codegen** — parse C++, emit `mission_ir.py`. ok (parses, no link) · consistent · single-source · works-without-run. Catch: `.ino`+MatrixMiniR4 include parse is fiddly.
3. **regex / structured-comment scrape** — grep `.goTo/.task` chains + task bodies, optional `// @ctps` anchors (same tech as existing Config.h regex parity). ok · consistent · single-source-ish · works-without-run. Catch: brittle to control flow/reformat.
8. **FFI/WASM blob** *(spec)* — CI cross-compiles logic to `.dll`/WASM, sim calls via ctypes. Real logic; heaviest plumbing, per-platform blob.

**B. One shared source, both consume**
6. **dual-valid `.def`** — `STEP(goTo,"A")` lines C++ `#include`s + ~20-line Python reader. ok · consistent (is C++) · one file · works-without-run. Cost: less ergonomic, weaker autocomplete.
7. **X-macro header** — `_registry[]` fits; branching task bodies don't.
9. **embedded mini-interpreter** *(spec)* — both run same opcode list. Large firmware blast radius.

**C. Invert source**
5. **author in Python → generate buildMission()** — mirror of `field_data.py→routes.h`. ok · single-source · works-without-run · BUT ADR-025 borderline: authored surface becomes Python, forfeits C++ DSL autocomplete/type-check.

**D. Parallel impl**
1b. **hand-mirror in Python** — extend runtime.py "MUST mirror" pattern. Lowest tech, fits repo, but dual-maintenance DRIFT on every edit.

**E. Secondary only**
10. **robot-emitted trace** *(spec)* — disqualified as primary (needs a run); natural LRS / diff-vs-reality path later.

Early read: #4 = fidelity/no-drift winner (runs real C++); #1b = fastest to ship (drifts); #2/#3 between.

### Compiler check (2026-06-07) — DECISIVE
Operator HAS a working host C++ toolchain (the long-standing "no local compiler" premise was WRONG):
- MSVC `cl.exe` 19.44.35222 (x64), VS Build Tools 2022; bundled cmake 3.31.6 + ninja 1.12.1; reached via `vcvars64.bat` (not on PATH — probe with PowerShell vswhere, not Bash).
- **Verified:** `WRO2026/test/` harness builds + passes **all 9 ctests locally with MSVC in 0.73 s** (CMakeLists already has an `if(MSVC)` branch).
→ Removes the only downside of mechanism #4 (no CI round-trip needed; runs locally, fast). Mem0 + local memory corrected.

### Converged ingestion choice → Mechanism #4 on the #46 foundation
`#46` (link real Tasks.cpp/Mission.cpp into the host harness) == the plumbing CTPS #4 needs. Build CTPS via #4 delivers #46 Phase 2 as a byproduct.
```
[ local host build: real Mission.cpp + Tasks.cpp + RECORDING mocks ]
   → buildMission() runs; recording mocks capture each goTo/grab/release/wait in order
   → dump ordered command stream → mission_trace.json  (derived artifact, ADR-025 OK like routes.h)
[ Python CTPS player ]
   • goTo targets → existing PathTrackerSim → trajectory "T" (top of map)
   • grab()/release()/wait → SIDE PANEL, time-aligned
   • smooth play / pause / seek over interpolated poses
```
Properties: single-source (real C++), zero-drift (executes real task bodies), works-without-robot (host build), ADR-025-consistent, runs locally. Cost: recording-mock layer = #46 Phase 2 upfront work. Alt: ship #1 hand-mirror first for a quick v1, migrate to #4 later (phasing decision for /plan).

### Issue #44 (pivot compensation) re CTPS
- NOT a v1 blocker — empty EDGES today (no pivots).
- Slight plus: CTPS reuses PathTrackerSim (advances by shortened distance_cm, snaps node) → would visually show the robot ending SHORT of a node marker on a leg starting at a pivot landing → surfaces #44 on screen (partial rebuttal to "tautology engine").
- Caveat: when pivots ship, trajectory rendering inherits #44's cross-module fix incl. sim parity; CTPS fidelity rides on sim staying parity-locked (already TC12/TC18).

### Task-internals handling (operator Q: tasks call Manipulator, Drive.driveDist, helpers)
Mechanism #4 handles arbitrary task internals BECAUSE recording happens at the hardware-facade boundary (Drive/PathTracker/Manipulator/Localization singletons), not the task-body level. Any call — direct, via helper, inside branches/loops — is captured in exact execution order the moment it touches a facade. This is why #4 beats #1/#3 (which would have to re-implement/re-parse helper logic + control flow).
- `Manipulator.*` → side panel, no chassis motion.
- `Drive.driveDist/turn/...` (raw move) → replayed through VirtualChassis; trajectory "T" is fed from TWO sources: PathTrackerSim (goTo hops) + raw Drive primitives.
- pure helpers (no hardware) → invisible (correct). `delay(ms)` → timeline pause / side-panel wait.
Requirements for /plan: (1) mock the FULL chassis-affecting facade set, not just PathTracker; (2) VirtualChassis replays recorded raw Drive moves alongside PathTrackerSim hops; (3) sensor-dependent branches resolve via deterministic SCRIPTED values from sensor mocks (camera is Phase 4) — CTPS shows that one branch (documented v1 limitation).

## Live direction

**Reframed by operator (2026-06-07):** CTPS = a *movie of the mission* the operator watches to confirm **authored logic is correct** (right tasks, right order, right destinations, manipulator commands at the right moments) **without a physical run** → saves dry-run time at competition. NOT a fault/physics predictor. Faults explicitly out of scope.

Requirements (operator's words):
- Show the trajectory **"T" at the top of the map**; that is the ONLY thing rendered on the map.
- A **side panel** narrates "what the robot is doing now" — e.g. `Manipulator command: grab()`, task name, current sub-command.
- **Play / stoppable / smooth** playback.
- Example mission: `task1 = goTo(A); getMosaic(); goTo(B); putMosaic()` / `task2 = goTo(C); getShovel(); goTo(D); putShovel()` → CTPS animates the route to A, side panel shows `grab()` etc.

Architecture that falls out:
```
CTPS = [ existing PathTrackerSim → trajectory "T" on map (top) ]
     + [ Tasks narration model   → side panel sub-commands ]
     + [ smooth animated player   → play / pause / seek over interpolated poses ]
```

Decisions settled by the reframe:
- **Trajectory source = reuse existing `PathTrackerSim`** (real NEXT_HOP route, parity-locked, free fidelity). Smooth play = interpolate poses between `step_through()` snapshots. No accel/timing model needed (not predicting physics).
- **Robot-EventTrace replay is OUT as primary input** (needs a run; defeats "without running"). LRS reuse demoted to a secondary adapter later, not a v1 driver. → defuses Critic #6.
- Defeats Critic #1 (no physics needed) and #5 (verifies *authoring* logic, which Mission queue-validation does NOT catch — only unknown task names).

Open fork → next: **task-narration source-of-truth** = how Python learns each task's ordered sub-command list for the side panel. (a) hand-mirror a tiny Python fn per task (fits existing parity pattern) vs (b) codegen from the C++ task body (single source, but no local C++ compiler). Manipulator *physics* NOT needed — only the command sequence.

## Killed / parked
- B6 browser/HTML player, B7 FSM time-travel debugger — out for v1 (narrower audience / toolchain widening); revisit if matplotlib playback proves too clunky.
- LRS tick-by-tick coupling — demoted to secondary adapter, not a v1 design constraint.
- "Predict physical problems" (collision/overshoot/slip oracle, B5) — out of v1 per operator; CTPS verifies authoring logic, not hardware behavior. B5 parked as a possible later CI gate.

## Killed / parked
<none yet>

## Decisions & debate verdicts
- **Ingestion = Mechanism #4** (host-build recording mocks → JSON), built on the #46 foundation, runs locally. Records at the hardware-facade boundary → robust to helpers/branches/loops calling Manipulator/Drive/etc.
- **Player substrate = /debate verdict Lean Yes (6.5/10) for HTML/Canvas**, gated on a cheap matplotlib-blitting spike first.
  - Smoothness split by gesture: PLAY both fine (matplotlib blitting 10–20×, sub-ms single artist); SCRUB matplotlib's Slider is the documented weak link, Canvas seek is O(1) → Canvas lower-risk on the operator's zero-lag-scrub bar.
  - Canvas "smoothness traps" (GC/throttle/JSON-freeze) mostly overstated at CTPS scale on a Windows desktop browser. Real costs of Canvas = untestable by parity suite + JS toolchain sprawl for solo dev + B6 was killed w/ a "measure matplotlib first" trigger.
  - **Gate:** build blitted-matplotlib prototype (≈1 afternoon, reuses visualizer.py) → if scrub smooth, stay matplotlib (save toolchain); if it stutters, go Canvas. Either way JS layer sits outside TC12/TC18.
