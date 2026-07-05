# CTPS: animate single-wheel pivot turns as a wheel arc (issue #55)

**Goal:** Make a single-wheel pivot turn legible in the CTPS HTML player by rendering the robot
box rotating **rigidly about the planted (braked) pivot wheel** through the turn angle — like the
real robot — and, in the same change, make a clean `goto` over a pivot **land on the node** (the
maneuver currently drifts ~12.7 cm and fires a spurious `drift_warn`).

**Approach:** A hybrid that keeps the exact-arc geometry in Python (provable) and uses the player's
reserved seam for a cosmetic overlay:
- **S1 — dense arc keyframes.** When `track_builder` detects that a `goto` takes a pivot turn, it
  replaces the sim's single in-place-turn keyframe with N keyframes (~1 per `ARC_STEP_DEG`) sampled
  along the exact rigid rotation about the planted wheel. Every player consumer — box marker,
  traveled-trail, trajectory polyline, hit-test — inherits the swing through the **existing pinned
  `poseAt` lerp** (no player-side interpolation change; sub-pixel accurate).
- **S4 — chassis nudge.** After the arc, `track_builder` sets the shared sim chassis center to the
  arc-end point so every downstream snapshot and the drift check are natively on-node. This is
  parity-safe: `_walk_hops` reads only `chassis.heading_deg` for dispatch, never x/y.
- **W overlay (cosmetic).** The Track gains an optional `pivots` segment list; `player_html` selects
  the reserved `wheel-arc` render mode when pivots are present; that mode draws the box at the
  standard lerped pose **plus** a transient planted-wheel pin + faint arc guide during the span.

The parity-locked sim, its primitive stream, and firmware↔sim parity are **untouched**; this is a
CTPS-visualization change only.

## Scope

- **In scope:**
  - `track_builder` detects a pivot on a `goto` (self-walks `next_hop` + `find_pivot_turn`) and emits
    dense arc keyframes for the rigid rotation about the planted wheel.
  - The chassis-nudge so a clean pivot `goto` lands on the node (drift ≈ 0, no spurious `drift_warn`),
    including chained pivots (e.g. U→V→W→X with pivots at V and W).
  - An optional `pivots` segment list in the Track schema (time-bracketed, baked world-cm wheel
    center), omitted entirely when empty.
  - `player_html` sets `render_mode` to `wheel-arc` only when the track carries ≥1 pivot; else `box`.
  - Register the `wheel-arc` render mode: box marker at the lerped pose + a transient planted-wheel
    pin and faint arc guide, shown only during a pivot span.
  - `ARC_STEP_DEG` config constant; tests (Python geometry + player_html + structural JS).
  - A new ADR (054) for the CTPS-pivot visualization + the chassis-nudge parity-safety convention.
- **Out of scope:**
  - Any change to the parity-locked sim primitive stream, `runtime.py` dispatch, or firmware↔sim
    parity. The sim keeps emitting the in-place `ACT_TURN_REL` + trimmed legs; the correction lives
    entirely in `track_builder`.
  - `trace_loader.py` and the C++ trace recorder — the pivot is **derived** from field `PIVOT_TURNS`,
    not authored in the command stream, so the recorder/loader "3-place contract" does not apply.
  - Player-side (JS) arc interpolation / any change to the pinned `poseAt` / `findKeyframeIndex`
    bisect (vetoed; the dense keyframes make it unnecessary).
  - REAR / reverse pivots (deferred to #54); the real `field_data.json` (`PIVOT_TURNS` stays empty).
  - Re-deriving or re-tuning `trim_cm` / `δ` (ADR-036 geometry is reused verbatim).

## Decisions

- **Hybrid over pure player-interpolation** — Fable's approach consult vetoed "player interpolates the
  arc exactly": if only the render mode did the arc, the trail + trajectory polyline would still lerp
  the chord and detach from the box ~10 px mid-90° pivot (the exact "not a real robot" artifact). Dense
  Python keyframes deliver the arc to every consumer through the one pinned lerp, sub-pixel, and the
  geometry is pytest-provable in Python instead of hard-to-test JS. User chose the hybrid.
- **Chassis nudge (S4) over a read-site offset accumulator (S5)** — the nudge keeps the module's
  "snapshot is the single authoritative pose" invariant by making the snapshot *correct*, one mutation
  point instead of N read sites. An accumulator would re-introduce the exact silent-drift failure class
  the `gotcha-sim-chassis-axle-on-node` memory warns against. Parity-safe: `_walk_hops` reads only
  `heading_deg` (runtime.py:2693), distances are graph-commanded, and the nudge happens while
  `nav_active=True` so the localization offset gate never sees it.
- **Time-bracketed segment with a baked world-cm center** — `pivots` entries use `t0`/`t1` (the
  player's native key, same bisect idiom as `findEventIndex`) and a pre-computed `(cx, cy)` wheel
  center, NOT keyframe indices (go stale under any future keyframe filtering) and NOT a
  wheel-offset derived player-side (that would re-import the Y-flip / heading-sign convention into a
  second place).
- **Byte-identical when no pivots** — `pivots` is omitted from `Track.to_dict()` when empty (no
  `schema_version` bump) and `render_mode` stays `box`, so a pivot-free trace's Track JSON + PLAYER_META
  are byte-identical to today. (The embedded JS assets necessarily change; "byte-identical" is about the
  emitted **data**, not the code.)
- **Transient overlay kept** — the `wheel-arc` mode draws a planted-wheel pin + faint arc guide during
  the span (transient, gone after the swing). This supersedes the earlier "no wheel dot" note, which was
  taken under the pure-W framing; the user explicitly opted into the overlay for the hybrid.
- **Approach consult outcome:** Fable — VETO on the exact-interpolation plank, recommended the hybrid
  (adopted). Codex — dispatched async in its own runtime (`task-mr7kichr-pminnb`), result not
  retrievable through the read-only forwarder; it gets an independent pass as a Phase 5.6 spec critic.

## Acceptance Criteria

- [ ] A `goto` whose route takes a declared pivot renders the box **rotating about the planted wheel**
      (center follows the arc of radius `track/2`, heading rotates through θ), with the wheel side
      matching `PivotTurnRecord.wheel` (left vs right).
- [ ] A clean `goto` over a single pivot lands the chassis **on the node** (drift ≤ `DRIFT_THRESHOLD_CM`,
      in practice ≈ 0) and emits **no** `drift_warn`. (Today it drifts 12.7 cm for a 90° pivot and warns.)
- [ ] A clean `goto` over **chained** pivots (U→V→W→X, pivots at V and W) animates each arc and lands on
      each node; the whole clean goto ends drift ≈ 0 with no `drift_warn`. (Today: 18 cm drift + warn.)
- [ ] The Track carries a `pivots` segment per taken pivot with the pinned shape; the segment's `wheel`,
      `cx`/`cy`, `theta_deg`, and `t0`/`t1` are correct.
- [ ] `player_html` emits `render_mode: "wheel-arc"` when the track has ≥1 pivot, `"box"` otherwise;
      the `wheel-arc` mode is registered so boot validation passes.
- [ ] When `field_data.PIVOT_TURNS` is empty (every current fixture + the real field), the emitted
      **Track JSON + PLAYER_META are byte-identical to today** (no `pivots` key, `render_mode: "box"`);
      all existing CTPS tests stay green.
- [ ] The sim primitive stream for a pivot route is unchanged (parity assertion): the same `ACT_TURN_REL`
      + trimmed-leg primitives as before the change.
- [ ] The pinned `poseAt` / `findKeyframeIndex` / `findEventIndex` / `wrapDeltaDeg` in `player.js` are
      untouched (TC12 stays green).

## Data Model (Track pivot segment)

The `Track` dataclass gains an optional `pivots` field (default empty list). `to_dict()` includes the
`"pivots"` key **only when non-empty**. `schema_version` stays `1` (additive optional field).

```python
# Each entry (one per taken pivot), sorted by t0:
{
    "t0": float,        # timestamp of the arc-start keyframe (approach-end pose)
    "t1": float,        # timestamp of the arc-end keyframe (post-swing pose)
    "cx": float,        # planted-wheel world x (cm)  -- rotation center
    "cy": float,        # planted-wheel world y (cm)
    "theta_deg": float, # signed swing angle (CCW +), = the ACT_TURN_REL delta
    "wheel": str,       # "left" (PIVOT_LEFT) | "right" (PIVOT_RIGHT)
}
```

Between `t0` and `t1` the dense keyframes already trace the arc through the existing lerp; the segment
exists so the `wheel-arc` render mode knows *when* a pivot is active and *where* the planted wheel is
(for the pin + arc guide). The keyframe list itself remains `{t, x, y, h}` — unchanged shape.

## Contracts & Interfaces

Single source of truth for every cross-task seam. Each entry names its owner and consumers.

### Shared types / schema

- **Track `pivots` list** — the exact shape above. Owner: `track_builder.py` (T2). Consumers:
  `player_html.py` (T3, presence check → render_mode) and `renderer.js` (T4, reads `TRACK.pivots`).
  Sorted by `t0`; omitted from `to_dict()` when empty.

### Signatures / conventions

- **Pivot detection signal** — owner T2 (internal to `build`): walk the goto's node path via
  `walk_next_hop(next_hop, from_node, target)`; for each interior triple `(u, v, w)` call
  `sim.find_pivot_turn(u, v, w)`. The pivot's turn is the `ACT_TURN_REL` primitive yielded while
  `sim.localization.current_node() == v`. Non-pivot pre-edge turns and the start/return turns miss
  (`find_pivot_turn` → `None`), so they render unchanged.
- **Arc geometry** — owner T2. With arc-start center `c0 = (x0, y0)` (previous keyframe), incoming
  heading `h0`, wheel side `s = +1` (left) / `-1` (right), `R = track_cm/2`
  (`planner.parse_track_cm_from_config()` — the SAME source ADR-036 derives `δ` from), and signed
  `θ` (the `ACT_TURN_REL` delta):
  ```
  W       = c0 + R·(cos(h0 + s·90°), sin(h0 + s·90°))      # planted wheel = rotation center
  N       = max(1, ceil(|θ| / ARC_STEP_DEG))
  for k in 1..N:  τ = k/N
      center_k  = W + Rot(τ·θ)·(c0 − W)                     # Rot(a)·(dx,dy)=(dx·cos a−dy·sin a, dx·sin a+dy·cos a)
      heading_k = normalize(h0 + τ·θ)
      Δt_k      = (|θ|/N) / SIM_TURN_DEG_S                  # pure-rotation time, total = |θ|/SIM_TURN_DEG_S
  c_end   = center_N                                        # == W + Rot(θ)·(c0 − W)
  ```
  Proven (analytically + numerically, 0.000 cm): `c_end` + the sim's existing trimmed departure leg
  lands the chassis center exactly on the node.
- **Chassis-nudge convention** — owner T2. After appending the N arc keyframes, set
  `chassis.x_cm, chassis.y_cm = c_end` on the shared `VirtualChassis` (heading already `h0+θ` from the
  sim's in-place turn). Safe because `_walk_hops` reads only `chassis.heading_deg` for dispatch
  (runtime.py:2693); x/y feed only the snapshot + drift check. Guard with a comment + a
  primitive-stream-unchanged regression assertion.
- **render_mode selection** — owner T3: `"wheel-arc" if track_dict.get("pivots") else "box"` in
  `player_html.render(...)` (replaces the hard-coded `"box"` at player_html.py:444).
- **Render-mode options bag** — owner T4: `_drawActors` passes `{fill, outline, sidePx, t: state.t}`
  to `mode.draw`. The existing `box` mode ignores the extra `t` (ES5-safe); `wheel-arc` reads it to
  find the active pivot span by bisecting `TRACK.pivots` on `[t0, t1]`.
- **`ARC_STEP_DEG`** — owner T1 (`ctps/config.py`), consumed by T2. Degrees per arc sub-keyframe;
  default `10.0` (sagitta at R=9 cm ≈ 0.14 px, sub-pixel).

### Naming (exact strings that must match across tasks)

- Track key: `"pivots"`; entry keys: `t0`, `t1`, `cx`, `cy`, `theta_deg`, `wheel`.
- `wheel` values: `"left"`, `"right"`. Render-mode id: `"wheel-arc"`. Config: `ARC_STEP_DEG`.

### File ownership

| File | Owner | Consumers |
|------|-------|-----------|
| `tools/path_planner/ctps/config.py` | T1 | T2 |
| `tools/path_planner/ctps/track_builder.py` | T2 | T5 |
| `tools/path_planner/ctps/player_html.py` | T3 | T6 |
| `tools/path_planner/ctps/assets/renderer.js` (+ token CSS) | T4 | T7 |

## Error Handling

- **Pivot record but route doesn't take the turn** — `find_pivot_turn` misses (returns `None`); render
  the turn unchanged (no arc, no segment). Covered by the existing TC17 "fires only when the route
  takes that turn" topology.
- **`|θ|` below one arc step** — `N = max(1, …)` guarantees ≥1 sub-keyframe; a legal pivot always clears
  `PIVOT_MIN_TURN_DEG` (validator), so this is defensive only.
- **`parse_track_cm_from_config()` fails** (missing `ConfigDrive.h`) — it raises loudly (no stale
  fallback, per issue #44). `track_builder` should surface, not swallow; a pivot-free build never calls
  it (no pivots detected → no arc), so pivot-free CTPS is unaffected.
- **Unknown / malformed `pivots` at the player** — the `wheel-arc` mode degrades to the plain box draw
  if a segment is missing `cx/cy/t0/t1` (draw the marker, skip the overlay); it never throws (one bad
  module must not abort the offline page, matching the registry's non-fatal contract).
- **`UnreachableGoalError` mid-goto** — unchanged; a pivot that was never reached emits no segment.

## Testing Strategy

**Levels:** Unit (Python track_builder geometry — the load-bearing part), Integration (player_html
emission), Structural (renderer.js source + emitted HTML — no JS execution harness exists; the e2e
subprocess runs the C++ dump, and the pinned player.js is verified by source-regex).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `goto` over the 90° dead-reckon pivot fixture (`make_dead_reckon_pivot_field`) | Unit | Final chassis pose on node W, drift ≈ 0, **no** `drift_warn` event (today: 12.7 cm + warn) |
| TC2 | Same goto: keyframes between approach-end and departure trace the arc about the planted wheel | Unit | ≥ `ceil(90/ARC_STEP_DEG)` intermediate keyframes; each `center_k` within tol of `W + Rot(τθ)(c0−W)`; heading rotates 90° |
| TC3 | Same goto: a `pivots` segment is emitted | Unit | One entry: `wheel == "right"` (θ<0), `(cx,cy)` = planted wheel, `theta_deg ≈ −90`, `t0 < t1` |
| TC4 | Chained pivots (`make_chained_pivot_field`, U→V→W→X) | Unit | Two arcs; lands on X; drift ≈ 0; no `drift_warn`; two `pivots` segments (today: 18 cm + warn) |
| TC5 | 70° LINE-incoming pivot (`make_line_incoming_pivot_field`) | Unit | Lands on L2 (drift ≈ 0); one `left`-wheel segment; arc present |
| TC6 | Pivot-free goto on the toy field (existing TC1 topology) | Unit | Keyframes still equal the independent `step_through` reference; **no** `pivots` key in `to_dict()` |
| TC7 | Sim primitive stream unchanged with pivots present | Unit | The `(primitive)` sequence for the pivot route == a reference `step_through` run (nudge touches only chassis x/y) |
| TC8 | `player_html.render` on a track WITH pivots | Integration | `PLAYER_META.render_mode == "wheel-arc"`; the `pivots` list is embedded in `TRACK` |
| TC9 | `player_html.render` on a pivot-free track | Integration | `render_mode == "box"`; embedded `TRACK` has no `pivots` key; byte-identical PLAYER_META vs today |
| TC10 | `renderer.js` registers the `wheel-arc` render mode | Structural | Source/emitted HTML contains a `CTPS.renderModes.register({ id: "wheel-arc" …})`; boot validation would accept it |
| TC11 | `ARC_STEP_DEG` present and positive | Unit | `ctps/config.py` exposes `ARC_STEP_DEG > 0` |

**Test data:** Reuse the existing `tools/path_planner/tests/fixtures/pivot_field_data.py` factories via a
synthetic `Trace([Command(op="goto", target=...)])` — no C++ `ctps_dump` needed. The pivot-free cases
reuse the toy field. The independent-`step_through` reference helper already in
`test_ctps_track_builder.py` is reused for TC6/TC7 (for pivot cases, assert on-node landing + segment,
NOT snapshot equality, since raw snapshots are the drifted in-place poses).

**Run commands:** `uv run --extra dev pytest tools/path_planner/tests/test_ctps_track_builder.py
tools/path_planner/tests/test_ctps_player_html.py tools/path_planner/tests/test_ctps_end_to_end.py`
(and the full `uv run --extra dev pytest tools/path_planner/tests/` before commit).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Add `ARC_STEP_DEG` config | — | low | `tools/path_planner/ctps/config.py` | Add `ARC_STEP_DEG: float = 10.0` with a docstring (degrees per pivot arc sub-keyframe; sagitta at R=9 cm ≈ 0.14 px). Satisfies AC "arc rendered"; consumed by T2. |
| T2 | track_builder: pivot detect + arc keyframes + chassis nudge + `pivots` schema | T1 | high | `tools/path_planner/ctps/track_builder.py` | Per **Contracts**: self-walk `next_hop`, detect the pivot turn (`ACT_TURN_REL` while `current_node()==v` and `find_pivot_turn(u,v,w)` set), append N arc keyframes (formula in Contracts), nudge `chassis.x/y = c_end`, append a `pivots` segment. Add `Track.pivots` field; `to_dict()` includes `"pivots"` only when non-empty. Use `parse_track_cm_from_config()` for R. Satisfies AC 1–4, 6, 7. |
| T3 | player_html: conditional `render_mode` | T2 | low | `tools/path_planner/ctps/player_html.py` | Replace hard-coded `"render_mode": "box"` (line 444) with `"wheel-arc" if track_dict.get("pivots") else "box"`. Track dict is already embedded verbatim, so `pivots` rides along. Satisfies AC 5, 6. |
| T4 | renderer: register `wheel-arc` mode + pin/arc-guide + `t` in options bag | T1 | med | `tools/path_planner/ctps/assets/renderer.js` (+ a `--ctps-*` token in the CSS assets) | Register `wheel-arc`: draw the box at `pose` (reuse the box path), then if `t` ∈ some `TRACK.pivots[i]` span, draw a small planted-wheel pin at `_project(cx,cy)` + a faint arc guide. Add `t: state.t` to the options bag in `_drawActors`. Read the pin/guide colour from a `--ctps-*` token (ADR-053 carve-out: no inline styles / no hard-coded hex). Do NOT touch the pinned `poseAt`/bisect. Satisfies AC 1, 5, 8. |
| T5 | track_builder pivot tests | T2 | low | `tools/path_planner/tests/test_ctps_track_builder.py` | TC1–TC7, TC11 above, using `pivot_field_data.py` fixtures via synthetic `Trace`. Assert on-node landing + no `drift_warn` + arc keyframes + `pivots` shape + pivot-free byte-identical + primitive-stream-unchanged. |
| T6 | player_html render_mode test | T3 | low | `tools/path_planner/tests/test_ctps_player_html.py` | TC8, TC9: `render_mode` selection + `pivots` embedded when present + byte-identical PLAYER_META when absent. |
| T7 | renderer structural + e2e test | T4 | low | `tools/path_planner/tests/test_ctps_end_to_end.py` (or `test_ctps_player_html.py`) | TC10: emitted HTML registers `wheel-arc`; a pivot track produces a self-contained page whose `render_mode` is `wheel-arc`. Structural (no JS execution). |
| T8 | ADR-054 + index | — | low | `docs/decisions/054-ctps-pivot-wheel-arc.md`, `DECISIONS.md` | Record: CTPS pivots are a visualization-only Track overlay (sim/firmware parity untouched); the chassis-nudge convention + its parity-safety; the time-bracketed `pivots` schema + `wheel-arc` mode; the Fable veto rationale (dense keyframes over player-side interpolation). Update the `DECISIONS.md` index + status table. |

T1 unblocks T2 and T4 (both parallel once the config constant lands; they touch different files and both conform to the pinned Contracts). T3 waits for T2's `pivots` schema. T5/T6/T7 each wait for their subject (T2/T3/T4) and can run in parallel. T8 has no code blocker (documents settled decisions) and can run any time.

## Notes for Implementer

- **The core correctness is the geometry, and it is Python-provable.** Land TC1–TC4 first; they are the
  whole point (kill the drift + show the arc). The JS overlay (T4) is cosmetic and has no execution
  test — verify it structurally and by opening a generated pivot page.
- **Why today drifts:** the sim renders a pivot as an in-place `ACT_TURN_REL` (center fixed) + legs
  trimmed by `δ = (track/2)·tan(|θ|/2)` on both sides. In-place spin drops the `track·sin(θ/2)` forward
  swing the real arc provides, so the chassis lands short. Rotating about the planted wheel restores
  exactly that displacement → lands on-node with the *unchanged* trims. Do not re-derive or re-tune `δ`.
- **Detection correlation gotcha:** the pivot's `ACT_TURN_REL` is the *pre-edge* turn of the departing
  hop `v→w`, emitted while `current_node()==v`. The start-hop turn and the return-prologue turn are also
  `ACT_TURN_REL` but miss `find_pivot_turn`, so gate on the record being non-`None`. Track the walked
  `(u, v, w)` from `walk_next_hop`; align it to the yielded stream by watching `current_node()`.
- **Nudge fence (parity):** `chassis.x_cm/y_cm` are plain assignable fields (see `_snapshot_chassis`).
  Mutate them only *between* generator yields, only for a detected pivot, with a comment citing
  runtime.py:2693 (dispatch reads heading only). TC7 (primitive-stream-unchanged) is the tripwire — if a
  future sim decision starts reading chassis x/y, TC7 fails loudly.
- **Keyframe timing:** give the whole arc the same visual duration the in-place turn had today
  (`|θ|/SIM_TURN_DEG_S`), split evenly across the N sub-keyframes (a turning box, not a driving one).
  Keep the running clock monotonic.
- **Byte-identical guard:** the pivot-free path must not append arc keyframes, must not call
  `parse_track_cm_from_config()`, and must omit the `pivots` key — so TC6/TC9 and every existing CTPS
  test stay green. This is the safety net for "output byte-identical when `PIVOT_TURNS` empty."
- **Rollback:** the change is additive and gated on a non-empty pivot set. Reverting T2 (or authoring no
  pivots) restores today's behavior exactly; T3/T4 are inert without a `pivots` list.
- **Base branch:** cut the #55 branch from `origin/main` (the local `feat/ctps-player-uiux-rework` tree
  is byte-identical to `origin/main` for `ctps/` + `sim/`, so nothing CTPS-related is lost).
