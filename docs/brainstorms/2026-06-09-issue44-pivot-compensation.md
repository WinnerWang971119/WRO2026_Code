# Brainstorm: issue #44 — pivot compensation writes a global following-edge distance

**Seed (verbatim):** Lets do issue#44

**Context summary:**
- Issue #44 (label: bug, OPEN, no comments): `apply_pivot_compensations`
  (`planner.py:146`) shortens the single following edge after a pivot and writes
  the shortened `distance_cm` back onto the *shared* graph edge, so it becomes the
  one global `EDGES[v][w]` value emitted to `routes.cpp`. `NEXT_HOP` is memoryless,
  so a route that stands at `v` and drives `v→w` *without* the preceding pivot
  under-drives. Latent today: production `field_data.py` has an empty `EDGES` list,
  so no pivot edges exist yet. Trigger-to-revisit: before the first pivot ships.
- Blast radius is narrow: the under-drive only reaches the wheels on
  **distance-terminated** following edges (`EDGE_BLIND` plain → `ACT_FORWARD`,
  `EDGE_PROJECTION` → `ACT_FOLLOW_DIST`). Sensor-terminated edges (`EDGE_LINE`
  junction stop; `EDGE_BLIND` `STOP_AT_LINE`) re-anchor on the physical line and
  are immune / masked. So: a pivot whose following edge is plain-blind or
  projection AND whose landing node is a mission start/goal.
- Second, deeper wrinkle: a pivot at a *line node* also interacts with the
  `LINE_OVERSHOOT` axle-centering advance (`LINE_SENSOR_AXLE_OFFSET_CM`). Current
  code only adjusts the *outgoing* `B→C` edge, not the incoming overshoot. A
  correct pivot at a line node must coordinate both — that's what makes the real
  fix cross-module (planner + validator + compiler + PathTracker + sim parity).
- mem0: sim must net exactly the edge length per hop (drift ≈ 0); LINE follow
  subtracts the axle offset and LINE_OVERSHOOT adds it back. Don't re-introduce
  per-junction offsets or relax `_POSITION_TOL_CM`. Validator enforces the
  one-forward-out-edge precondition on pivot landings (finding #36). There is a
  pattern of deferring real-but-latent bugs in unused features via a tracking
  issue rather than a speculative guard.

**Three fix directions already in the issue:**
1. Path-specific compensation — carry the shortened distance on the `u→v→w`
   transition, not the global `v→w` edge. Correct for all routes but `NEXT_HOP`
   carries no predecessor → needs compiler/runtime pivot-context per hop. Largest,
   likely its own phase.
2. Scoped validator guard — reject/warn a pivot whose landing `v` is a leg
   boundary (`START_NODE`) AND whose single following edge resolves to a
   distance-terminated primitive. Cheap; encodes the exact narrow condition;
   cannot see runtime goals.
3. Line-node overshoot coupling — when pivot source/landing is a junction node,
   fold the `LINE_OVERSHOOT` advance into the pivot geometry.

**Clarified framing:** Pivots are imminent (the first real pivot edge in
`field_data.py` is about to be authored), so the latent bug is about to go live
and needs a working fix before that lands. Appetite is undecided — let the
perspective round surface cost/benefit. **Key reframing from the user, verified
geometrically:** a pivot at node `v` is a *two-sided* perturbation. The axle
chord (radius `track/2` about one wheel, along the turn bisector) decomposes
symmetrically into the incoming and outgoing drive directions, so BOTH `u→v` and
`v→w` must undershoot by `(T/2)·tan(θ/2)`. The current code compensates only the
outgoing edge `v→w` (`next_edge_data`), and never the incoming `u→v`
(`pivot_edge_data` is read-only). Two corollaries: (a) the existing magnitude
`T·sin²(θ/2)` is only correct at exactly θ=90° — at other angles it
under-compensates even the one edge it touches (correct per-edge amount is
`(T/2)·tan(θ/2)`); (b) fixing the incoming side via a global `EDGES[u][v]` write
would reintroduce the #44 aliasing bug on the incoming edge — so a pivot creates
TWO context-dependent corrections, both of which break the memoryless global-edge
model, strengthening the case for per-transition (`u→v→w`) compensation. The sim
(`tools/path_planner/sim/`) has NO pivot model (`grep pivot` empty), so sim-robot
parity for this geometry is currently vacuous.

## Perspectives (round 1)

### Critic
1. The geometry the fix "corrects" is a fiction the firmware never executes uniformly — `PathTracker.cpp:136-139` has THREE turn modes (`COUNTER_ROT` default zero-translation, `PIVOT_LEFT`, `PIVOT_RIGHT`); the (T/2)tan(θ/2) derivation only holds for a true single-wheel pivot about a specific braked wheel, and real braked-wheel pivots scrub so the pivot point wanders. Deriving a closed-form correction for an unmeasured maneuver.
2. Two physical chassis with unknown, different track widths; the whole correction scales linearly with `TRACK_CM`, which is a hard-coded placeholder `15.0` (planner.py:84, "Update when real chassis measured"). Residual from a wrong TRACK_CM dwarfs the sin²-vs-tan difference. Polishing a quantity whose first sig-fig is unknown.
3. The "incoming edge must also undershoot" half collides with LINE_OVERSHOOT + the hard sim-parity constraint. For a pivot landing on a line node the incoming edge terminates on the physical line, then LINE_OVERSHOOT dead-reckons a fixed offset (runtime.py:114); the incoming correction there is "reduce LINE_OVERSHOOT," a different mechanism from "shorten distance_cm," and it's the exact FSM transition the team already burned itself on (CTPS playback). Must edit firmware + sim byte-identically.
4. Per-transition (u→v→w) context is a cross-module NEXT_HOP rewrite for an EDGES list that is empty today — textbook premature generalization; CLAUDE.md rule 9 forbids speculative abstractions; CI SHA-gates routes.
5. The validator one-out-edge rule (finding #36) already launders the bug; the genuinely dangerous case (pivot landing = leg boundary feeding a single distance-terminated edge) is exactly the scoped-validator-guard condition. The full rewrite spends its budget on cases already immune/masked/forbidden.
6. "Both edges undershoot symmetrically" hides a cross-track residual and a heading error no distance correction can cancel; competition chassis closes heading on IMU and distance on odometry — two sensors the offline closed-form conflates. Makes the longitudinal term look rigorous while lateral/angular stay wrong → false confidence.

### Feasibility — VERDICT: Conditional
GEOMETRY VERIFICATION: **CLAIM A CONFIRMED** by independent first-principles derivation. Δ = (T/2)(I−R(θ))n̂, |Δ|=T·sin(θ/2) (matches code `chord_cm`). Decomposing Δ in the NON-orthogonal incoming/outgoing drive basis gives EXACTLY (T/2)tan(θ/2) along EACH direction → both adjacent edges shorten by (T/2)tan(θ/2). The split is EXACT — **no cross-track residual once both drive directions are the basis**, because Δ lies in their span. (A single-direction projection DOES leave a residual — so my earlier "(T/2)sinθ projection" framing was a red herring; the right operation is the two-direction decomposition.) **CLAIM B CONFIRMED**: code uses T·sin²(θ/2)=(T/2)(1−cosθ); correct is (T/2)tan(θ/2); they coincide ONLY at θ=90° (all =T/2; at 60° → 0.25T vs 0.289T). Code is correct by accident at 90°, doubly wrong otherwise (wrong magnitude on the edge it touches, silently zero on the incoming edge).
Hidden assumptions: pivot point = braked-wheel contact, reference = axle center (if "node" is the line-sensor location it leads the axle by LINE_SENSOR_AXLE_OFFSET_CM → radius ≠ T/2); no scrub/slip; exact θ; inner-vs-outer wheel sets sign not magnitude.
Hard blockers: none absolute, BUT the full per-transition fix is blocked by the memoryless NEXT_HOP contract unless that contract changes.
Key solvable challenge / finding: **node-splitting converts the context-dependent correction into a context-free edge property**, sidestepping the memoryless blocker — split v into pivot/non-pivot virtual nodes so the pivot route traverses a dedicated edge owning the correction; costs an extra node + validator rule, breaks nothing in the walk/sim/SHA gate. Magnitude fix is a one-line change in compute_pivot_compensation (planner.py:141), independently shippable, auto-propagates to sim (runtime.py has NO pivot logic, just reads baked distance_cm). With T=15, correction at 90°=7.5cm, 45°≈3.1cm — same order as the ±15cm STOP_AT_LINE window, so on sensor-terminated edges it's inside the masking margin; bites only on chained distance-terminated edges.

### Thinker (branches)
1. **Fold the correction into the turn primitive, not the edge** — emit a richer `ACT_PIVOT_REL(theta,side,track)` whose handler rolls the axle forward/back so the maneuver itself nets zero translation; EDGES stays pristine, no route under-driven.
2. **Synthetic intermediate node at the pivot apex** — graph_builder splices a node v' at the true post-pivot axle position; u→v' and v'→w become real straight edges, point-turn model exact everywhere, memoryless table needs no special case. Bug becomes structurally impossible. (Feasibility independently reached this = node-splitting.)
3. **Runtime re-anchor: pivots only into sensor-terminated edges** — validator forbids a distance-terminated edge right after a pivot; the next physical line swallows the error. Blast-radius insight as a routing rule. Pair with an ADR.
4. **Per-(predecessor,node,successor) trim table, post-Dijkstra** — flip ordering: after NEXT_HOP is chosen, walk realized 3-tuples and emit a sparse `PIVOT_TRIM[u][v][w]` side-table the FSM consults; EDGES stays global+unaliased; handles both sides + arbitrary θ. Heavier table.
5. **Change the maneuver: point-turn-about-axle-center (tank turn)** — counter-rotation spins about axle center → zero translation → existing geometry already exact, compensation returns 0 by construction. If chassis can tank-turn in the contested spots, DELETE the whole compensation subsystem (code + finding #36 + buggy formula) instead of fixing it. Single-wheel pivot becomes opt-in only where tank-turn is mechanically blocked.
6. **CNC tool-radius comp: runtime offset, planner stays radius-agnostic** — store ideal axle-center paths; a thin runtime comp layer applies the (track/2) offset. Changing chassis track needs NO route regeneration (runtime constant) → dodges the SHA/CI coupling.
7. **Two/three-point K-turn that returns to the node** (speculative) — forward-pivot + reverse-pivot whose translations cancel; net ≈0, point-turn model holds, zero table changes; safe fallback maneuver.
8. **Validator-only dissolve: ban the config, defer the math** (speculative) — EDGES empty today, so delete the θ=90°-only formula and reject pivot-before-distance-edge with a precise error; buy time until real angle + track are measured.
9. **Dubins/arc-turn replaces the point-turn idealization** (speculative) — model every turn as a finite-radius arc (kinematic reality); the (T/2)tan(θ/2) error disappears because it was only the point-vs-arc discrepancy. Largest blast radius; only branch that makes the world model match real kinematics.

## Live direction
**Node-split (synthetic apex node)** — chosen by the user. Splice a synthetic
node at the true post-pivot axle position so the pivot's two-sided
`(T/2)·tan(θ/2)` correction becomes ordinary straight-edge geometry; NEXT_HOP
stays memoryless, bug structurally impossible, handles any θ and both sides.
Two agents converged on this independently (Thinker #2 = Feasibility node-split).

**Prerequisite sub-task surfaced (source-cited): single-source the track width.**
`Config.h:178 TRACK_MM = 180.0f` (18 cm, marked `TODO: measure on real chassis`)
vs `planner.py:84 TRACK_CM = 15.0` (15 cm) — the hand-maintained Python mirror is
ALREADY stale (its own comment claims Config is 150 mm; it's actually 180 mm). A
20% track error feeds directly into every `(T/2)tan(θ/2)` correction and node-split
bakes it into routes.cpp; the sim mirrors it without an independent check. Fix the
dual-source drift (ideally code-gen TRACK from one source) before/with node-split.

### Node-split open design questions (to resolve in the handoff)
1. One synthetic node or two? The pivot shortens BOTH approach (start (T/2)tan(θ/2)
   before v) AND departure (end (T/2)tan(θ/2) past v): entry point A = v −
   (T/2)tan(θ/2)·ĥ_in, exit point B = v + (T/2)tan(θ/2)·ĥ_out. Capturing both
   exactly wants two synthetic points (A,B) with the pivot turn between them.
2. A synthetic node has NO physical line to detect — so edges touching it cannot be
   EDGE_LINE/ACT_FOLLOW_INT (sensor-terminated) and the robot cannot re-anchor on
   it. If v itself is a real line junction, node-split changes where/whether
   LINE_OVERSHOOT re-anchoring happens. THIS is the crux to resolve.
3. Sim parity: sim has no pivot model. If the pivot translation is fully baked into
   the (shortened) adjacent straight edges and the turn is modeled in-place
   (zero-translation, as today), sim + robot net correctly — but only if the
   synthetic node coords are chosen so the in-place-turn model lands right. Must be
   byte-identical and preserve per-hop net-length = edge-length (drift ≈ 0).
4. Validator finding #36 (pivot landing has exactly one forward out-edge) and the
   junction-type classification both assume nodes = detectable junctions; synthetic
   nodes need their own validator rules / exemptions.

## Killed / parked

## Decisions & debate verdicts
