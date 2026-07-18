# ADR-072: Reverse (REAR-traversal) single-wheel pivots — force-SAME direction latch + runtime wheel-flip policy

**Status:** Implemented
**Date decided:** 2026-07 (issue #54, Phase 3)

## Rule

A single-wheel pivot may now fire when its edges are traversed in **reverse** (REAR),
not just FRONT. **A pivot is single-direction**, enforced by **force-SAME**: the pivot
commits to the direction resolved for its **approach** hop (`u→v`) and forces the
departure hop (`v→w`) to that same direction. This is the direct generalization of the
old force-FRONT (ADR-036), which forced both legs FRONT unconditionally.

The wheel and the overshoot budget are resolved **at runtime** by the shared
header-only `ReversePivotPolicy.h` (`WRO2026/src/nav/`), mirrored by a Python twin in
`sim/runtime.py` and consumed by CTPS:

- `resolvePivotWheel(storedForwardWheel, dir)` — flips `PIVOT_LEFT ↔ PIVOT_RIGHT` when
  `dir == Direction::REAR`; unchanged for `FRONT`/`AUTO`.
- `pivotOvershootBudgetCm(dir)` — returns `Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM` for
  `REAR`, else `Config::LINE_SENSOR_AXLE_OFFSET_CM`.

The generated `PivotTurn[]` table (ADR-036) is **unchanged** — it still stores the
forward-sense wheel and the direction-invariant `δ`. The flip is a pure runtime mapping
applied at dispatch, single-sourced through the one policy so firmware, the Python sim,
and CTPS cannot disagree. `routes.h`/`routes.cpp`, the emitter goldens, and the
pivot-free real field stay byte-identical.

## Why

Reverse pivots exist to let the planner's `Direction::AUTO` resolution
(`BIDIRECTIONAL_AUTO_ENABLED`, ADR-024) actually choose the cheaper travel direction
through a pivot node, the same way it already does for a plain LINE hop. Before this
ADR, a pivot was FRONT-only (ADR-036): even when the auto-resolved direction for the
approach or departure hop was REAR — because it avoided a large in-place spin, or let
the rear sensor land the seek — the pivot forced a FRONT traversal anyway, silently
overriding the direction the rest of the route just resolved.

Lifting that restriction outright would let a pivot's two legs resolve independently
and pick opposite directions, which has no single body rotation that satisfies both —
the wheel-flip and the `δ` trim are derived from one coherent `(u, v, w)` turn, not two
unrelated hops. Force-SAME is the generalization that keeps the maneuver geometrically
coherent while still letting the pivot go reverse: it lets the *approach* hop resolve
normally, then pins the *departure* hop to that same direction, exactly as the old
force-FRONT pinned both legs to FRONT unconditionally.

## Wheel-flip derivation

Restating ADR-036's sign(θ) → wheel chain with the reverse-roll step added:

1. **Forward (FRONT, ADR-036):** body rotation `θ = wrap(h_out − h_in)`; the driven
   (unbraked) wheel rolls **forward**; `PIVOT_LEFT ⇒ θ > 0`, `PIVOT_RIGHT ⇒ θ < 0`;
   `δ = (T/2)·tan(|θ|/2)`.
2. **Reverse (REAR, force-SAME ⇒ both legs REAR):** the robot arrives at body heading
   `h_in + 180°` and leaves at `h_out + 180°` (both legs traverse REAR, so the same
   180° offset applies to each end), so body rotation is `wrap((h_out+180) − (h_in+180))
   = wrap(h_out − h_in) = θ` — the **+180° offsets cancel**, and `θ` is unchanged in
   **both magnitude and sign** from the forward case.
3. **The driven wheel rolls backward.** A REAR traversal means the previously-driven
   side is now the trailing side rolling in reverse. To sweep the nose through the
   *same* signed `θ`, the chassis must brake the **opposite** wheel from the one the
   forward mapping would plant — the same body rotation, produced by the opposite
   contact point, because the roll direction at each wheel has inverted.
4. **Wheel flips; `δ` does not.** `resolvePivotWheel` therefore maps
   `PIVOT_LEFT ↔ PIVOT_RIGHT` on `REAR` and leaves `δ = (T/2)·tan(|θ|/2)` untouched —
   the magnitude depends only on `|θ|` and the track width, neither of which changes
   under force-SAME.

**Do not derive the wheel from the live commanded delta** — the `PivotTurn[]` table is
the wheel authority; the commanded-delta sign is only a runtime tripwire
(`PathTracker.cpp`), never the source of truth (this reintroduces the ADR-036
Blocker-1 disagreement bug if inverted).

## Conservative per-pinning budget

A pivot's sensor-terminated **approach** leg budgets `δ` against a direction-resolved
offset, not always the FRONT one:

- **Reverse-reachable** (2-board chassis, not FRONT-pinned by a per-edge pin or the
  pipeline default) → budget against `LINE_SENSOR_AXLE_OFFSET_REAR_CM` (**7.68 cm**, an
  unmeasured placeholder pending the wired rear board, `ConfigDrive.h`).
- **FRONT-pinned or reverse-impossible** (sensor-in on a 1-board chassis) → budget
  against `LINE_SENSOR_AXLE_OFFSET_CM` (**12.02 cm**).

This has a geometric consequence worth stating explicitly: with `δ = (T/2)·tan(|θ|/2)`
and `T/2 ≈ 9 cm` (`Config::TRACK_MM = 180`), a 90° pivot has `δ ≈ 9 cm`, which already
**exceeds** the 7.68 cm REAR budget. Reverse pivots are therefore geometrically limited
to turns of roughly **θ < ~81°** (vs ~106° forward, ADR-044). A sharper reverse pivot
is **rejected offline** by the validator, naming the escape: pin `direction:"front"` on
both legs, soften the turn, or approach via a dead-reckon BLIND leg instead of a
sensor-terminated one. `direction:"front"` on both pivot legs remains the force-forward
escape hatch — the v1 behavior is still fully reachable, just no longer the only one.

## Faults / rejects

- **Binding direction contradiction → `Fault::PIVOT_DIRECTION_MISMATCH`.** If a
  *binding* higher-precedence direction override on the departure hop — a per-edge
  `direction_override`, the pipeline direction default, **or a builder-level route/
  arrival override on the last hop** — resolves to a direction that contradicts the
  pivot's approach direction, force-SAME cannot reconcile the two legs. The validator
  rejects any **statically-visible** contradiction (per-edge pin or pipeline default)
  offline; the runtime fault is the backstop for a builder-level override, which the
  validator cannot see ahead of time. The dispatch faults rather than running a
  mis-trimmed maneuver.
- **REAR seek into a pivot on a 1-board chassis → `Fault::REVERSE_UNAVAILABLE`
  (pivot-scoped).** A new check at the seek dispatch fires only when
  `_approachingPivot.active` and the resolved seek direction is REAR with
  `LINE_BOARD_COUNT < 2` — added **without widening** the existing `reverseUnavailable`
  helper, which would regress the documented non-pivot forced-REAR `STOP_AT_LINE` →
  dead-reckon-reverse degrade.
- **Reverse-reachable reset-node (wall-zero) pivot must be FRONT-pinned.** The
  wall-zero interlude drives forward off the wall to `WALL_PIVOT_CLEARANCE_CM`, sized
  for the FRONT sweep radius; a REAR departure would sweep the tail arc close to the
  wall, untested. The validator rejects a reverse-reachable reset-node pivot unless
  both its legs are FRONT-pinned.
- **`δ > budget` reached at runtime → `Fault::PIVOT_TRIM_UNDERFLOW`** (unchanged from
  ADR-044) — the floor-clamp backstop for any path the validator's static analysis
  cannot see.

## Known limitation

The validator's board-impossibility check keys off the **incoming** leg's edge type. A
1-board chassis with a pinned-REAR **BLIND** approach into a **SENSOR** (LINE/seek)
departure is not statically rejected by that check. This case is left to the firmware
`REVERSE_UNAVAILABLE` / dead-reckon-reverse backstop at runtime — it never silently
mis-pivots, but it is not caught offline. Closing this gap fully is out of scope here;
it does not regress anything v1 already covered.

## Arc-end geometry (CTPS / sim)

The approach lands the axle `δ` **short** of `v`. The rigid rotation about the planted
wheel then ends the axle at **`v + δ·û_out(travel)`** — `δ` **past** `v` along the
outgoing travel ray, not at `v` itself. The departing dead-reckon trim subtracts that
same `δ`, netting the chassis onto `w`; on a sensor-terminated departure the
destination line re-anchors it instead (ADR-054's sensor-departure re-anchor,
mirrored on the departure side). This holds for both FRONT and REAR traversal — the
arc-end is never on `v`.

## How to apply

- **Firmware:** `PathTracker`'s pivot dispatch forces the departure hop's direction to
  match the resolved approach direction (force-SAME, generalizing the old force-FRONT);
  the wheel and overshoot budget are resolved at runtime by the shared header-only
  `ReversePivotPolicy.h` (`resolvePivotWheel`, `pivotOvershootBudgetCm`) in
  `WRO2026/src/nav/`.
- **Python sim:** `sim/runtime.py` mirrors the same policy so firmware and sim cannot
  disagree on the resolved wheel or overshoot budget.
- **Validator:** the reverse-reachability overshoot-budget check and its rejects
  (sharper-than-~81° reverse pivot, a binding direction contradiction, a
  reverse-reachable reset-node pivot not FRONT-pinned) run offline against
  `LINE_SENSOR_AXLE_OFFSET_REAR_CM`.
- **CTPS:** the arc render (ADR-054) consumes the sim's taken-pivot resolved direction
  to flip the planted-wheel side and pin for a REAR-traversed pivot.
- **Authoring escape hatch:** pin `direction:"front"` on both legs of a pivot to keep it
  forward-only — above the REAR budget, or whenever the reverse maneuver isn't wanted —
  reproducing the v1 force-FRONT behavior exactly.

## Trigger to revisit

- The physical **B3.8-reverse** bench (Roadmap §13) and `Config::PIVOT_SCRUB_FACTOR`
  are deferred until the rear board is wired — no scored mission may rely on a reverse
  pivot hop until B3.8-reverse passes.
- `δ > reverse-budget` reached via a path the validator cannot statically reject —
  pick up the reverse "re-seat" / signed-overshoot generalization tracked in
  **issue #75** (unchanged scope from ADR-044).
- The known limitation above (board-impossibility check keyed off the incoming leg
  only) is revisited if a field ever authors a 1-board pinned-REAR BLIND→SENSOR pivot
  chain — currently unreached by the real field.
- `LINE_SENSOR_AXLE_OFFSET_REAR_CM` is measured on the physical rear board — re-run the
  validator's REAR overshoot-budget check against every reverse-reachable pivot.

## See also

- [ADR-035 — Two-sided pivot compensation `δ = (T/2)·tan(θ/2)`](./035-two-sided-pivot-compensation.md) — the δ proof this ADR reuses unchanged.
- [ADR-036 — Per-turn pivot compensation `(u, v, w)`](./036-per-turn-pivot-compensation.md) — the `PivotTurn[]` table and sign(θ)→wheel derivation this ADR restates with the reverse-roll step; its FRONT-only v1 restriction is lifted here.
- [ADR-044 — STOP_AT_LINE-incoming pivot trim](./044-stop-at-line-incoming-pivot-trim.md) — the seek-overshoot reduction this ADR extends with the direction-resolved budget; the ~106° FRONT limit this ADR's ~81° REAR limit parallels.
- [ADR-054 — CTPS pivot wheel-arc](./054-ctps-pivot-wheel-arc.md) — the arc render this ADR's REAR geometry extends (planted-wheel side + travel-frame departure ray on REAR).
- [ADR-024 — Bidirectional line following](./024-bidirectional-line-following.md) — the direction-resolution machinery (`Direction::{AUTO,FRONT,REAR}`, `resolveDirection`) force-SAME pins.
- `docs/plans/2026-07-19-issue54-reverse-pivots.md` — the originating plan.
- Issue #54 (closed by this), #75 (reverse re-seat for `δ > budget`, unchanged).
