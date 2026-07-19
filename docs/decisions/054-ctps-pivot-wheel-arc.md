# ADR-054: CTPS pivot wheel-arc — a visualization-only Track overlay with a parity-safe chassis nudge

**Status:** Implemented
**Date decided:** 2026-07 (issue #55)

## Rule

A single-wheel pivot turn is rendered in the CTPS HTML player as the robot box rotating
**rigidly about the planted (braked) pivot wheel** through the turn angle, instead of
spinning in place. This is a **`track_builder`-only** correction: the parity-locked sim
(`sim/runtime.py`), its primitive stream, and firmware↔sim parity are untouched. Three
mechanisms make this true:

1. **Dense arc keyframes.** When `track_builder` detects a pivot on a `goto`, it replaces
   the sim's single in-place-turn keyframe with `N = max(1, ceil(|θ|/ARC_STEP_DEG))`
   keyframes sampled along the exact rigid rotation about the planted wheel. Every player
   consumer (box marker, trail, trajectory polyline, hit-test) inherits the swing through
   the existing pinned `poseAt` lerp — no player-side interpolation change.
2. **Chassis nudge.** After emitting the arc, `track_builder` sets the shared sim
   chassis's `x_cm`/`y_cm` to the arc-end point so every downstream snapshot and the
   drift check land on-node. This is the load-bearing parity-safety claim of this ADR
   (see below) — it is a **post-hoc correction of the recorded pose**, not a change to
   how the sim computes or dispatches anything.
3. **`pivots` Track segment + `wheel-arc` render mode.** The `Track` schema gains an
   optional, time-bracketed `pivots` list (omitted entirely when empty — byte-identical
   emission for every pivot-free trace, which today is every fixture and the real field).
   `player_html` selects the reserved `wheel-arc` render mode only when `pivots` is
   non-empty; that mode draws the box at the standard lerped pose plus a transient
   planted-wheel pin and faint arc guide during the pivot's `[t0, t1]` span.

## Why

**The drift bug.** The sim renders a pivot as an in-place `ACT_TURN_REL` (rotation center
fixed) plus legs trimmed by `δ = (T/2)·tan(|θ|/2)` on both sides (ADR-035/036). An
in-place spin drops the `track·sin(θ/2)` forward swing the real single-wheel pivot
produces, so the recorded chassis pose lands short of the node — 12.7 cm for one 90°
pivot, 18 cm across a chained U→V→W→X pair, each firing a spurious `drift_warn`. Rotating
the *rendered* pose about the planted wheel restores exactly that missing displacement,
with the existing, unchanged `δ` trims.

**Why a nudge in the CTPS builder, not a fix in the sim itself.** The arc-about-wheel
could instead be modeled inside the sim's own `_emit_turn_to` pivot case, which would fix
`go_to`, `step_through`, *and* `demo.py` at once and land every consumer on-node
natively. This was rejected: it would mutate the parity-locked sim's believed pose,
forcing a rebaseline of every sim-pose golden and breaking firmware↔sim parity
(ADR-050) — the exact thing issue #55 commits to leaving alone. Confining the correction
to `track_builder` leaves the primitive stream and parity provably intact (TC7: the same
`chassis.x/y` mutation applied mid-`step_through` does not change the collected primitive
sequence versus an unperturbed reference run).

**Why dense Python keyframes, not player-side (JS) arc interpolation.** The alternative —
teach `poseAt`/`findKeyframeIndex` to interpolate the arc exactly, keeping the sim's
single in-place-turn keyframe — was vetoed on approach consult (Fable): if only the
render mode drew the arc, the trail and trajectory polyline would still lerp the straight
chord between the two flanking keyframes and visibly detach from the box mid-swing (~10 px
at a 90° pivot) — a "not a real robot" artifact for exactly the geometry this issue exists
to fix. Baking dense keyframes in Python instead delivers the arc to *every* consumer
through the one pinned lerp, sub-pixel accurate, and the geometry is pytest-provable
(TC1–TC4) rather than hard-to-test JS. The pinned `poseAt`/`findKeyframeIndex`/
`findEventIndex`/`wrapDeltaDeg` in `player.js` stay byte-untouched (TC12).

**The chassis-nudge parity-safety proof.** A direct write to `chassis.x_cm`/`y_cm` cannot
smuggle a behavior change into the sim, for two independent reasons:

- (a) **Dispatch never reads chassis x/y.** `_walk_hops` resolves the next hop's direction
  via `resolve_direction(..., current_heading=self._chassis.heading_deg, ...)`
  (`runtime.py:2693`) — heading only. Every commanded distance comes from the graph
  (edge length ± the fixed `δ` trims), never from the chassis's recorded position. An x/y
  write therefore cannot perturb what gets dispatched next.
- (b) **A raw chassis write never reaches `LocalizationSim`.** The operator-offset
  integrator (ADR-042's sim analogue) grows only through
  `LocalizationSim.accumulate_operator_move`, whose sole caller is
  `VirtualChassis`/`operator_move` (`runtime.py:2230`). Setting `chassis.x_cm`/`y_cm`
  directly bypasses that call path entirely — it mutates the field the snapshot code and
  drift check read, and nothing else observes it.

## How to apply

- **Detection — one-hop lookahead, mirroring firmware (ADR-036).** `track_builder`
  tracks the previously-visited node during the existing `sim.step_through(target)`
  iteration. When an `ACT_TURN_REL` primitive is yielded while
  `sim.localization.current_node() == v`, resolve `u` (the tracked predecessor) and
  `w = next_hop[v][target]` — a single, `None`-safe dict index, **never**
  `walk_next_hop` (which raises `UnreachableGoalError` eagerly and would drop the
  reachable-prefix keyframes on a partial/`UNREACHABLE` goto). Then call
  `sim.find_pivot_turn(u, v, w)`; a miss (spawn turn with no `u`, terminal node with
  `w == None`, non-pivot turn, or empty `PIVOT_TURNS`) renders the turn unchanged. This
  literally mirrors the firmware key `findPivotTurn(cur, nxt, getNextHop(nxt, dest))`
  (ADR-036), so pivot-free builds (today, every fixture and the real field) never touch
  the arc path at all.
- **Arc geometry.** With arc-start center `c0` (the previous keyframe), incoming heading
  `h0`, wheel side `s = +1` (left) / `-1` (right), `R = track_cm/2`
  (`planner.parse_track_cm_from_config()` — the same source ADR-036 derives `δ` from),
  and signed `θ` (the `ACT_TURN_REL` delta), the planted wheel
  `W = c0 + R·(cos(h0+s·90°), sin(h0+s·90°))` is the rotation center; `N` sub-keyframes
  sample `center_k = W + Rot(τθ)·(c0−W)` and `heading_k = normalize(h0+τθ)` for
  `τ = k/N`, with even pure-rotation timing (`Δt_k = (|θ|/N)/SIM_TURN_DEG_S`, same total
  duration the in-place turn had). This is proven exact (0.000 cm) for a
  distance-terminated departure.
- **Sensor-terminated departure (ADR-044 D1).** A sensor-terminated departure hop (LINE-
  out, or a stop-at-line seek) takes no departing trim in the sim, so driving the full
  distance from `c_end` leaves a residual of exactly `δ` along the departure edge ray —
  purely along-edge, no lateral component. The physical robot re-anchors that away on the
  destination line (ADR-044 D1's "re-anchors at v" incoming-side mechanism, mirrored here
  on the departure side); `track_builder` reproduces it by landing the departure keyframe
  on the destination node instead of the dead-reckoned point. The arc itself stays
  continuous — this is a downstream sensor re-anchor, not a pivot-arc snap.
- **Chassis-nudge mechanics.** After appending the N arc keyframes, `track_builder` sets
  `chassis.x_cm, chassis.y_cm = c_end` on the shared `VirtualChassis` (heading is already
  `h0+θ` from the sim's own in-place turn). Guard the mutation with a comment citing
  reasons (a) and (b) above; TC7 is the executable tripwire that a chassis-position write
  mid-`step_through` cannot change the collected primitive sequence.
- **Reset-node pivot.** If an `ACT_WALL_*` reset interlude fired at `v` between the
  approach and the turn (arrival at a `"reset"`-typed node re-anchors pose unconditionally,
  `runtime.py`'s wall-reset dispatch ahead of the next hop's plan), the arc-start premise
  `c0` is already stale — `track_builder` skips the arc and emits no segment, rendering
  the plain in-place turn (the reset already re-established the pose).
- **`pivots` Track segment.** One entry per taken pivot:
  `{t0, t1, cx, cy, theta_deg, wheel}` (`wheel` is `"left"`/`"right"`, mapped from the
  `TurnMode` enum `PIVOT_LEFT`/`PIVOT_RIGHT` — the enum never crosses into the emitted
  JSON). `t0`/`t1` are the player's native bisect key (same idiom as `findEventIndex`);
  `cx`/`cy` is the pre-computed planted-wheel world-cm center. Deliberately **not**
  keyframe indices (go stale under any future keyframe filtering) and **not** a
  wheel-offset re-derived player-side (would re-import the Y-flip/heading-sign convention
  into a second place). `Track.to_dict()` includes `"pivots"` only when non-empty;
  `schema_version` stays `1` (additive optional field) — a pivot-free trace's Track JSON
  and `PLAYER_META` are byte-identical to before this change.
- **`render_mode` selection.** `player_html.render()` sets
  `"wheel-arc" if track_dict.get("pivots") else "box"` (replacing a hard-coded `"box"`).
  The `wheel-arc` render mode is registered in `renderer.js`'s render-mode registry
  (ADR-053): it draws the box at the standard lerped pose, then — while `t` falls inside
  a `pivots[i]`'s `[t0, t1]` span — a transient planted-wheel pin (`--ctps-accent` token)
  and a faint arc guide (`--ctps-muted` token), reusing existing CSS custom properties per
  ADR-053 rather than introducing a new token or CSS asset. A `pivots` segment missing
  `cx`/`cy`/`t0`/`t1` degrades to the plain box draw (never throws), matching the
  registry's non-fatal contract.
- **`ARC_STEP_DEG`** (`ctps/config.py`, default `10.0°`, sagitta at `R=9 cm` ≈ 0.14 px)
  is the only new tunable; it governs sub-keyframe density, never the geometry itself.

## Trigger to revisit

- REAR / reverse pivots (deferred to #54) need CTPS rendering — the arc geometry above
  assumes FRONT-only traversal, matching ADR-036's v1 scope. **Implemented by
  [ADR-073](./073-reverse-pivots.md) (#54):** `pivots[]` gains a `direction` field and
  the planted-wheel side + pin flip for REAR, and the sensor-departure re-anchor uses
  the travel-frame departure ray (`depart_rad` flipped 180°) rather than the body-frame
  ray, which points opposite travel on REAR.
- The real `field_data.json` gains a non-empty `PIVOT_TURNS` — this ADR's
  byte-identical-when-empty guarantee then no longer covers the shipped field, and the
  `wheel-arc` path becomes load-bearing for the competition trace, not just fixtures.
- A future feature needs to mutate chassis pose mid-`step_through` for a reason other than
  a detected pivot — re-derive the parity-safety proof for that case rather than assuming
  reasons (a)/(b) generalize; they were checked specifically for a position-only,
  heading-unchanged write.
- The sim's own turn dispatch (`_emit_turn_to`) is ever changed to model the arc natively
  — this ADR's "nudge over sim fix" tradeoff should be re-weighed against the (by then
  smaller) golden-rebaseline cost.

## See also

- [ADR-035 — Two-sided pivot compensation `δ = (T/2)·tan(θ/2)`](./035-two-sided-pivot-compensation.md) — the trim geometry this ADR's arc restores without re-deriving.
- [ADR-036 — Per-turn pivot compensation `(u, v, w)`](./036-per-turn-pivot-compensation.md) — the one-hop-lookahead detection this ADR mirrors (`findPivotTurn`), and the wheel-sign derivation `pivots[].wheel` reuses.
- [ADR-044 — STOP_AT_LINE-incoming pivot trim (D1)](./044-stop-at-line-incoming-pivot-trim.md) — the "re-anchors at v" sensor mechanism this ADR's sensor-departure re-anchor mirrors on the departure side.
- [ADR-050 — PathTracker sim↔firmware FSM parity](./050-pathtracker-sim-parity.md) — the parity guarantee this ADR is careful not to disturb; TC7 is this issue's own parity tripwire.
- [ADR-053 — CTPS player modular assets + extension registries](./053-ctps-player-modular-shell.md) — the `renderModes` registry `wheel-arc` registers into, and the CSS-custom-property convention the pin/arc-guide reuse (`--ctps-accent`/`--ctps-muted`), unmodified.
- [ADR-033 — CTPS ingests a derived mission trace](./033-ctps-derived-mission-trace.md) — confirms `trace_loader.py`/the C++ recorder are untouched: the pivot is derived from `PIVOT_TURNS`, not authored in the command stream.
- [ADR-073 — Reverse (REAR-traversal) single-wheel pivots](./073-reverse-pivots.md) — lands the REAR arc rendering this ADR deferred; the sim exposes the taken pivot's resolved direction, consumed here through the one shared `ReversePivotPolicy`.
- `docs/plans/2026-07-05-ctps-pivot-wheel-arc.md` — the originating plan.
- Issue #55 (closed by this), #54 (reverse pivots, closed by ADR-073), #44 (per-turn pivot compensation).
