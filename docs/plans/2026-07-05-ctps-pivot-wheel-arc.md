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
  the `gotcha-sim-chassis-axle-on-node` memory warns against. Parity-safe for two verified reasons:
  (a) `_walk_hops` reads chassis state only via `heading_deg` (runtime.py:2693) for dispatch (distances
  are graph-commanded), so an x/y write cannot change the primitive stream; and (b) a direct
  `chassis.x_cm/y_cm` assignment **never reaches `LocalizationSim`** — the operator-offset integrator
  grows only through `accumulate_operator_move` (sole caller `operator_move`), which a raw chassis write
  does not invoke. (An earlier draft mis-stated the reason as a `nav_active` gate; that was wrong — the
  offset gate is irrelevant because the write never feeds the integrator at all.)
- **Nudge (consumer-side) over fixing the sim's own turn dispatch** — the arc-about-wheel could instead
  be modeled inside the sim's `_emit_turn_to` pivot case, which would fix `go_to`, `step_through`, AND
  `demo.py` at once and land every consumer on-node natively. Rejected: it mutates the **parity-locked
  sim's believed pose**, forcing a rebaseline of every sim-pose golden and breaking the "sim untouched"
  scope fence this issue commits to (visualization-only). The nudge confines the correction to the CTPS
  builder and leaves firmware↔sim parity provably intact. (Recorded in ADR-054.)
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
      in practice ≈ 0) and emits **no** `drift_warn`, for **both** a distance-terminated departure (exact
      via the arc + trim) **and** a sensor-terminated departure (the along-edge δ residual is absorbed by
      the destination line re-anchor — see Error Handling). (Today it drifts 12.7 cm for a 90° pivot and warns.)
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
      + trimmed-leg primitives as before the change. (Note: the issue's "goto endpoints unchanged" means
      the **logical node + primitive stream** are unchanged; the Track's *rendered pose* for a pivot route
      intentionally changes so it lands on-node — that change IS AC2, and is not a contradiction.)
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

- **Pivot detection signal** — owner T2 (internal to `build`). **Per-turn one-hop lookahead, mirroring
  the firmware** (ADR-036 `findPivotTurn(cur, nxt, getNextHop(nxt, dest))`) — do NOT pre-walk the whole
  path. During the existing `sim.step_through(target)` iteration, track the previously-visited node.
  When an `ACT_TURN_REL` primitive is yielded while `sim.localization.current_node() == v`, resolve
  `u = ` the tracked predecessor, `w = next_hop[v][target]` (a **single, `None`-safe dict index** — NOT
  `walk_next_hop`, which raises `UnreachableGoalError` eagerly), then call `sim.find_pivot_turn(u, v, w)`.
  A miss (`None` — non-pivot turn, the spawn turn with no predecessor, the terminal node where
  `w == None`, or empty `PIVOT_TURNS`) renders the turn unchanged. This rides the untouched
  `step_through` loop, so **nothing raises eagerly** and today's partial-track / `UNREACHABLE` behavior
  (reachable-prefix keyframes + the `UNREACHABLE` event) is preserved byte-for-byte, including pivots the
  robot takes on the reachable prefix of an unreachable goto.
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
  Proven (analytically + numerically, 0.000 cm) **for a DISTANCE-terminated departure** (BLIND /
  dead-reckon): `c_end` + the sim's existing δ-trimmed departure leg lands the chassis center exactly on
  the node. **A SENSOR-terminated departure** (a LINE-outgoing edge, or a stop-at-line seek) is
  validator-legal and takes NO departing trim in the sim (it re-anchors on the destination line), so
  driving the full distance from `c_end` leaves a residual of exactly `δ` **along the departure edge
  ray** (`û₁`) — purely along-edge, no lateral component. The physical robot re-anchors that away on the
  destination line (ADR-044 D1), so for a sensor-terminated departure the departure keyframe **lands on
  the destination node** (the along-edge re-anchor) rather than the dead-reckoned `c_end + L·û₁`; the arc
  itself stays continuous (this is a downstream sensor re-anchor, not a pivot-arc snap). See Error Handling.
- **Chassis-nudge convention** — owner T2. After appending the N arc keyframes, set
  `chassis.x_cm, chassis.y_cm = c_end` on the shared `VirtualChassis` (heading already `h0+θ` from the
  sim's in-place turn). Safe for two reasons: (a) `_walk_hops` reads chassis state only via
  `heading_deg` for dispatch (runtime.py:2693) — x/y feed only the snapshot + drift check, so a write
  cannot change the primitive stream; and (b) a raw `chassis.x_cm/y_cm` write never reaches
  `LocalizationSim` (the offset integrator grows only through `accumulate_operator_move`). Guard with a
  comment citing (a)+(b) and the TC7 primitive-stream-unchanged tripwire. **Sensor-departure exception:**
  when the departure hop is sensor-terminated (regime `line`/`sensor`), the departure keyframe re-anchors
  to the **destination node** instead (the along-edge δ residual the sim would otherwise dead-reckon) —
  the arc still ends at `c_end`, only the departure's landing is the node.
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
- **Enum→string conversion is owned by T2**: `PivotTurnRecord.wheel` is a `TurnMode` enum
  (`PIVOT_LEFT`/`PIVOT_RIGHT`), not a string; T2 maps it to the emitted `"left"`/`"right"` in the segment.
- The pin + arc guide **reuse the existing `--ctps-accent` (pin) and `--ctps-muted` (guide) tokens**
  already resolved in `renderer.js` — **no new CSS token / no CSS-asset edit** (keeps T4 to `renderer.js`).

### File ownership

| File | Owner | Consumer tasks |
|------|-------|----------------|
| `tools/path_planner/ctps/config.py` | T1 | T2 |
| `tools/path_planner/ctps/track_builder.py` | T2 | T3, T5 |
| `tools/path_planner/ctps/player_html.py` | T3 | T6 |
| `tools/path_planner/ctps/assets/renderer.js` | T4 | T7 |
| `tools/path_planner/tests/test_ctps_track_builder.py` | T5 | — |
| `tools/path_planner/tests/fixtures/pivot_field_data.py` | T5 | — |
| `tools/path_planner/tests/test_ctps_player_html.py` | T6 | — |
| `tools/path_planner/tests/test_ctps_end_to_end.py` | T7 | — |
| `docs/decisions/054-ctps-pivot-wheel-arc.md`, `DECISIONS.md` | T8 | — |

## Error Handling

- **Pivot record but route doesn't take the turn** — `find_pivot_turn` misses (returns `None`); render
  the turn unchanged (no arc, no segment). Covered by the existing TC17 "fires only when the route
  takes that turn" topology.
- **`|θ|` below one arc step** — `N = max(1, …)` guarantees ≥1 sub-keyframe; a legal pivot always clears
  `PIVOT_MIN_TURN_DEG` (validator), so this is defensive only.
- **`parse_track_cm_from_config()` fails** (missing `ConfigDrive.h`) — it raises loudly (no stale
  fallback, per issue #44). `track_builder` should surface, not swallow; a pivot-free build never calls
  it (no pivots detected → no arc), so pivot-free CTPS is unaffected.
- **Sensor-terminated pivot departure** (LINE-outgoing / stop-at-line seek from the pivot node) — the
  sim takes no departing trim and dead-reckons `δ` **along the departure edge** past the node; the real
  robot re-anchors on the destination line (ADR-044 D1), so `track_builder` lands the departure keyframe
  **on the destination node**. Because the residual is purely along-edge (no lateral component, proven in
  Contracts), this re-anchor is exact and leaves no visible discontinuity. Keeps AC2 (on-node, no
  `drift_warn`) for this topology. A dedicated LINE-departing fixture + TC covers it (T5).
- **Pivot at a reset node** — arriving at a `"reset"`-typed pivot node fires the wall-reset interlude
  (runtime.py:3157), which absolutely snaps chassis x/y+heading and emits `ACT_WALL_*` keyframes *before*
  the departing turn — destroying the `c0` arc-start premise. Handling: if an `ACT_WALL_*` primitive
  appeared at `v` between the approach and the turn, `track_builder` **skips the arc + emits no segment**
  and renders the plain in-place turn (the reset has already re-established the pose). The implementer
  must also confirm whether the validator should forbid a reset-typed pivot node outright; file a
  follow-up if it currently allows it and the skip path is untested.
- **Unknown / malformed `pivots` at the player** — the `wheel-arc` mode degrades to the plain box draw
  if a segment is missing `cx/cy/t0/t1` (draw the marker, skip the overlay); it never throws (one bad
  module must not abort the offline page, matching the registry's non-fatal contract).
- **`UnreachableGoalError` mid-goto** — unchanged. The one-hop-lookahead detection rides the untouched
  `step_through` loop, so pivots the robot *takes on the reachable prefix* still render their arc +
  segment; only the dead-end-and-beyond turns emit nothing (their `w` is unreachable → `None` → miss).

## Testing Strategy

**Levels:** Unit (Python track_builder geometry — the load-bearing part), Integration (player_html
emission), Structural (renderer.js source + emitted HTML — no JS execution harness exists; the e2e
subprocess runs the C++ dump, and the pinned player.js is verified by source-regex).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `goto` over the 90° dead-reckon pivot fixture (`make_dead_reckon_pivot_field`) | Unit | Final chassis pose on node W, drift ≈ 0, **no** `drift_warn` event (today: 12.7 cm + warn) |
| TC2 | Same goto: keyframes trace the arc about the planted wheel, with correct timing | Unit | `N = ceil(90/ARC_STEP_DEG)` sub-keyframes replacing the single turn keyframe (so `N−1` *intermediate* keyframes before the arc-end); each `center_k` within tol of `W + Rot(τθ)(c0−W)`; heading rotates through 90°; total arc duration `== 90/SIM_TURN_DEG_S` split evenly; clock strictly monotonic across them |
| TC3 | Same goto: a `pivots` segment is emitted | Unit | One entry: `wheel == "right"` (θ<0), `(cx,cy)` = planted wheel, `theta_deg ≈ −90`, `t0 < t1` |
| TC4 | Chained pivots (`make_chained_pivot_field`, U→V→W→X) | Unit | Two arcs; lands on X; drift ≈ 0; no `drift_warn`; two `pivots` segments (today: 18 cm + warn) |
| TC5 | 70° LINE-incoming pivot (`make_line_incoming_pivot_field`) | Unit | Lands on L2 (drift ≈ 0); one `left`-wheel segment; arc present |
| TC6 | Pivot-free goto on the toy field (existing TC1 topology) | Unit | Keyframes still equal the independent `step_through` reference; **no** `pivots` key in `to_dict()` |
| TC7 | Nudge does not perturb the sim primitive stream | Unit | Iterate `sim.step_through` on the pivot fixture and apply the SAME `chassis.x/y` mutation the nudge does mid-iteration; the collected primitive sequence `==` an unperturbed reference `step_through` run. (Cannot read primitives from `Track` — `build()` discards them — so the test drives `step_through` directly.) |
| TC8 | `player_html.render` on a track WITH pivots | Integration | `PLAYER_META.render_mode == "wheel-arc"`; the `pivots` list is embedded in `TRACK` |
| TC9 | `player_html.render` on a pivot-free track | Integration | Structural equality vs an expected literal: `render_mode == "box"`; embedded `TRACK` has no `pivots` key; `PLAYER_META == the expected dict`. (True byte-vs-today is verified once manually — see Notes — since a test can't diff against pre-change code.) |
| TC10 | `renderer.js` registers `wheel-arc` + the span-guard exists | Structural | Source/emitted HTML contains `CTPS.renderModes.register({ id: "wheel-arc" …})` AND a `TRACK.pivots`/`t0`/`t1` span presence-check guard (source-regex), so the degrade-on-missing-segment path is asserted present |
| TC12 | `goto` over a **LINE-departing** pivot fixture (new: sensor-terminated departure) | Unit | Lands on the destination node (drift ≈ 0), no `drift_warn`; the departure re-anchors on the node per the sensor-departure rule; one segment emitted |
| TC11 | `ARC_STEP_DEG` present and positive | Unit | `ctps/config.py` exposes `ARC_STEP_DEG > 0` |

**Test data:** Reuse the existing `tools/path_planner/tests/fixtures/pivot_field_data.py` factories via a
synthetic `Trace([Command(op="goto", target=...)])` — no C++ `ctps_dump` needed. **TC12 needs a new
`make_line_departing_pivot_field()` factory** in `pivot_field_data.py` (owned by T5): a pivot whose
departure `v→w` is a LINE edge into a line node (sensor-terminated departure), which no existing fixture
has; it must pass `validate`. The pivot-free cases reuse the toy field. The independent-`step_through`
reference helper already in `test_ctps_track_builder.py` is reused for TC6/TC7 (for pivot cases, assert
on-node landing + segment, NOT snapshot equality, since raw snapshots are the drifted in-place poses).

**Run commands:** `uv run --extra dev pytest tools/path_planner/tests/test_ctps_track_builder.py
tools/path_planner/tests/test_ctps_player_html.py tools/path_planner/tests/test_ctps_end_to_end.py`
(and the full `uv run --extra dev pytest tools/path_planner/tests/` before commit).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Add `ARC_STEP_DEG` config | — | low | `tools/path_planner/ctps/config.py` | Add `ARC_STEP_DEG: float = 10.0` with a docstring (degrees per pivot arc sub-keyframe; sagitta at R=9 cm ≈ 0.14 px). Satisfies AC "arc rendered"; consumed by T2. |
| T2 | track_builder: pivot detect + arc keyframes + chassis nudge + `pivots` schema | T1 | high | `tools/path_planner/ctps/track_builder.py` | Per **Contracts**: detect the pivot turn by the **one-hop lookahead** (`ACT_TURN_REL` while `current_node()==v`; `w = next_hop[v][target]` None-safe; `find_pivot_turn(u,v,w)`) — NO `walk_next_hop` pre-walk. Append N arc keyframes (formula in Contracts) with even pure-rotation timing; nudge `chassis.x/y = c_end` for a distance departure, or land the departure keyframe on the destination node for a **sensor departure**; skip the arc when an `ACT_WALL_*` reset interlude fired at `v`. Append a `pivots` segment (map `TurnMode`→`"left"/"right"`). Add `Track.pivots` field; `to_dict()` includes `"pivots"` only when non-empty. Call `parse_track_cm_from_config()` for R only on the pivot path. Satisfies AC 1–4, 6, 7. |
| T3 | player_html: conditional `render_mode` | T2, T4 | low | `tools/path_planner/ctps/player_html.py` | Replace hard-coded `"render_mode": "box"` (line 444) with `"wheel-arc" if track_dict.get("pivots") else "box"`. Track dict is already embedded verbatim, so `pivots` rides along. Blocked by T4 so `wheel-arc` is registered before any page emits `render_mode: "wheel-arc"` (else boot.js:85 logs a validation error). Satisfies AC 5, 6. |
| T4 | renderer: register `wheel-arc` mode + pin/arc-guide + `t` in options bag | — | med | `tools/path_planner/ctps/assets/renderer.js` | Register `wheel-arc`: draw the box at `pose` (reuse the box path), then if `t` ∈ some `TRACK.pivots[i]` span, draw a small planted-wheel pin at `_project(cx,cy)` (token `--ctps-accent`) + a faint arc guide (token `--ctps-muted`) — reuse existing tokens, NO new CSS / no CSS-asset edit (ADR-053 carve-out: no inline styles / hard-coded hex). Add `t: state.t` to the options bag in `_drawActors`. Do NOT touch the pinned `poseAt`/bisect. Conforms to the Contracts `pivots` schema. Satisfies AC 1, 5, 8. |
| T5 | track_builder pivot tests + LINE-departing fixture | T2 | low | `tools/path_planner/tests/test_ctps_track_builder.py`, `tools/path_planner/tests/fixtures/pivot_field_data.py` | TC1–TC7, TC11, TC12, using existing `pivot_field_data.py` fixtures via synthetic `Trace`, PLUS a new `make_line_departing_pivot_field()` factory (must pass `validate`) for TC12. Assert on-node landing (distance AND sensor departure) + no `drift_warn` + arc keyframes + timing + `pivots` shape + pivot-free no-`pivots`-key + TC7 primitive-stream-unchanged (via direct `step_through`). |
| T6 | player_html render_mode test | T3 | low | `tools/path_planner/tests/test_ctps_player_html.py` | TC8, TC9: `render_mode` selection + `pivots` embedded when present + expected-literal PLAYER_META when absent. |
| T7 | renderer structural + e2e test | T4 | low | `tools/path_planner/tests/test_ctps_end_to_end.py` | TC10: emitted HTML registers `wheel-arc` + the span guard; a pivot track produces a self-contained page whose `render_mode` is `wheel-arc`. Structural (no JS execution). |
| T8 | ADR-054 + index | — | low | `docs/decisions/054-ctps-pivot-wheel-arc.md`, `DECISIONS.md` | Record: CTPS pivots are a visualization-only Track overlay (sim/firmware parity untouched); the chassis-nudge convention + its **correct** parity-safety rationale (dispatch reads only `heading_deg`; raw x/y writes never reach `LocalizationSim` — NOT a `nav_active` gate); the one-hop-lookahead detection (mirrors firmware ADR-036); the sensor-departure re-anchor (ADR-044 D1); the time-bracketed `pivots` schema + `wheel-arc` mode; and why the nudge was chosen over fixing the sim's own turn dispatch (parity/golden-rebaseline) and over player-side interpolation (Fable veto). Update the `DECISIONS.md` index + status table. |

T1 unblocks T2. T4 has no blocker and runs in parallel with T1/T2 (different file, conforms to the pinned Contracts). T3 waits for **both** T2 (the `pivots` schema in the Track) and T4 (the `wheel-arc` mode must be registered before a page emits `render_mode: "wheel-arc"`). T5/T6/T7 each wait for their subject (T2/T3/T4) and can run in parallel. T8 has no code blocker (documents settled decisions) and can run any time.

## Notes for Implementer

- **The core correctness is the geometry, and it is Python-provable.** Land TC1–TC4 first; they are the
  whole point (kill the drift + show the arc). The JS overlay (T4) is cosmetic and has no execution
  test — verify it structurally and by opening a generated pivot page.
- **Why today drifts:** the sim renders a pivot as an in-place `ACT_TURN_REL` (center fixed) + legs
  trimmed by `δ = (track/2)·tan(|θ|/2)` on both sides. In-place spin drops the `track·sin(θ/2)` forward
  swing the real arc provides, so the chassis lands short. Rotating about the planted wheel restores
  exactly that displacement → lands on-node with the *unchanged* trims. Do not re-derive or re-tune `δ`.
- **Detection = one-hop lookahead, NOT a pre-walk.** The pivot's `ACT_TURN_REL` is the *pre-edge* turn of
  the departing hop `v→w`, emitted while `current_node()==v`. Track the predecessor `u` during the
  `step_through` loop; at the turn, resolve `w = next_hop[v][target]` — a single `None`-safe dict index,
  **never `walk_next_hop`** (it raises `UnreachableGoalError` eagerly and would drop the reachable-prefix
  keyframes + change the `UNREACHABLE` label even with `PIVOT_TURNS` empty). Then `find_pivot_turn(u,v,w)`;
  the start-hop turn (no `u`), the return-prologue turn, the terminal node (`w == None`), and non-pivot
  turns all miss (`None`) and render unchanged. This literally mirrors the firmware key
  `findPivotTurn(cur, nxt, getNextHop(nxt, dest))` (ADR-036).
- **Sensor-terminated departure re-anchor.** For a distance departure, nudge `chassis.x/y = c_end` and the
  trimmed leg lands on-node. For a **sensor** departure (LINE-out / seek — the sim takes no departing
  trim), the residual is exactly `δ` **along the departure edge** (no lateral part); land the departure
  keyframe on the destination node (the physical line re-anchor, ADR-044 D1). TC12 + the new
  `make_line_departing_pivot_field()` fixture cover it.
- **Reset-node pivot.** If an `ACT_WALL_*` reset interlude fired at the pivot node `v` (arrival at a
  `"reset"`-typed node), it already snapped the pose — skip the arc and emit no segment there. Confirm
  whether the validator should forbid a reset-typed pivot node; file a follow-up if it doesn't.
- **Nudge fence (parity):** `chassis.x_cm/y_cm` are plain assignable fields (see `_snapshot_chassis`).
  Mutate them only *between* generator yields, only for a detected pivot, with a comment citing the two
  reasons it's safe: dispatch reads only `heading_deg` (runtime.py:2693), and a raw x/y write never reaches
  `LocalizationSim` (offset grows only via `accumulate_operator_move`). **Do NOT** cite a `nav_active` gate
  — that reasoning is wrong (the write never feeds the integrator). TC7 is the tripwire.
- **Keyframe timing:** give the whole arc the same visual duration the in-place turn had today
  (`|θ|/SIM_TURN_DEG_S`), split evenly across the N sub-keyframes (a turning box, not a driving one).
  Keep the running clock monotonic.
- **Byte-identical guard:** on the pivot-free path, do not append arc keyframes, do not call
  `parse_track_cm_from_config()`, and omit the `pivots` key — so TC6/TC9 and every existing CTPS test stay
  green. The detection *lookup* itself is byte-safe (a pure `.get` that misses on empty `PIVOT_TURNS`); only
  the arc EMISSION is gated. **Verify AC6 once by hand:** generate a pivot-free page before and after the
  change and `diff` the emitted `TRACK` + `PLAYER_META` JSON — they must be identical (a test can't diff
  against pre-change code).
- **Rollback:** the change is additive and gated on a non-empty pivot set. Reverting T2 (or authoring no
  pivots) restores today's behavior exactly; T3/T4 are inert without a `pivots` list.
- **Base branch:** cut the #55 branch from `origin/main` (the local `feat/ctps-player-uiux-rework` tree
  is byte-identical to `origin/main` for `ctps/` + `sim/`, so nothing CTPS-related is lost).
