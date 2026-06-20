# ADR-043: Wall-zero reset nodes — a new node kind that re-anchors pose against a known wall

**Status:** Implemented
**Date decided:** 2026-06 (Phase 2)

## Rule

A `"reset"` node kind carries a parallel **sparse** `RESET_NODES{node, axis, wall_coord, facing}`
table (emitted like the `PivotTurn[]` table of ADR-036, not a per-node field on `NODES[]`).
On arrival at a reset node, PathTracker runs a three-phase interlude:

1. **`WALL_STOP`** — one latched `Drive.wallStop`, pushing **rear-first** (`.setReverse(true)`):
   only the chassis rear may take wall load — a front press would break the mechanism. The
   primitive's SQUARE phase first turns in place to `facing` (front pointing AWAY from the wall, so
   the rear faces it), then presses straight in `DriveMode::RAW` (open-loop + motor bias; a
   heading-hold PID would only fight the wall as one wheel stalls first) with **back-off 0**.
   The stall reuses the stop-at-line no-progress watchdog (ADR-027) with the meaning **inverted** —
   a stall is *success*. A push that never stalls within `Config::WALL_STOP_TIMEOUT_MS` records a
   **non-latching** advisory (`Drive.wallStopTimedOut()`); it does not halt the FSM.
2. **`REANCHOR`** — one atomic `Localization.reanchorAtWall(axis, wall_coord, facing)` that squares
   IMU heading to `facing` **and** re-zeros the chosen axis to the **wheel axle**, at
   `wall_coord - sign(wall_coord - node_coord)·Config::WALL_CONTACT_OFFSET_CM` — NOT the wall
   surface. The rear contact face is at the wall; the tracked axle sits `WALL_CONTACT_OFFSET_CM` in
   front of it, on the node side. Only the on-axis coordinate + heading are corrected; the off-axis
   position is untouched (the wall gives no cross-axis information).
3. **`WALL_DEPART`** — `driveDist(standoff - WALL_CONTACT_OFFSET_CM)` **forward** (GYRO, holding the
   squared heading) back onto the node, then `clearOffset()` so the final belief is the node
   coordinate. This gives a following pivot clearance — the robot never turns pressed against the
   wall — and rides on the re-anchor's baseline re-snapshot, so it is a fresh encoder move immune to
   the slip the stall induced.

**Authoring contract:** `facing` is the **parked heading** (front away from the wall; the rear at
`facing + 180` presses it). The validator rejects a `facing` pointing toward the wall and rejects a
node-to-wall standoff below `Config::WALL_PIVOT_CLEARANCE_CM` (pivot sweep radius + safety,
hand-mirrored into `validator.py`), so both the SQUARE turn and the post-zero pivot clear the wall.

An **off-by-default** strict wall-normal flag (`Config::WALL_NORMAL_STRICT_ENABLED == false`)
escalates a bad post-stop heading to `Fault::WALL_NOT_SQUARE`; with the flag off a skewed stop is
trusted and re-squared anyway.

## Why

Reset nodes kill accumulated encoder/IMU drift against a physical ground truth — the wall —
**without new hardware** (detection is encoder-stall, not a bumper). This reconciles with ADR-001:
wall-zero is **bounded calibration**, not continuous wall-following — a discrete ground-truth
re-anchor, the same family as an intersection crossing. The rear-first press, the axle-margin
re-anchor, and the return-to-node departure exist because (a) only the rear can take wall load on
this chassis, (b) the tracked point is the axle, a fixed margin behind the contact face, so
re-zeroing to the wall surface would be optimistic by that margin, and (c) the chassis cannot pivot
while pressed, so it must return to a node authored with turn clearance. The re-anchor corrects ONE
axis + heading only; cross-axis drift survives. The manipulator/box pre-stage at the wall and the
"don't wall-zero while carrying" guard are deferred to **issue #77**.

**Amends ADR-005 (the second amendment).** `reanchorAtWall` is a **new writer** of `_yawOffsetDeg`
and of the position axis state, alongside `zeroHeadingFromCurrent` and `declarePosition`, so it
**supersedes ADR-005's "yaw offset written ONLY by `zeroHeadingFromCurrent`" wording**. The
single-**writer** rule holds: only `Localization`'s own methods mutate pose — the write *set* grows,
the writer *module* does not. ADR-042 was the first amendment; this is the second.

## How to apply

- **Authoring:** a `"reset"` node in `field_data` with `facing` AWAY from the wall and standoff
  ≥ `WALL_PIVOT_CLEARANCE_CM`. The pipeline emits a sparse `Routes::ResetNode[]` read via
  `FieldMap::getResetInfo(node)` (ADR-028); `axis` is `0 = x`, `1 = y`.
- **Drive:** `Drive.wallStop(cm).setReverse(true).setBackOff(0.0f).setMode(RAW)` — rear-first,
  stall-terminated; advisory via `Drive.wallStopTimedOut()`.
- **Localization:** `reanchorAtWall(axis, wallCoord, headingDeg)` — atomic; re-zeros the axis to the
  axle (`wallCoord - sign·WALL_CONTACT_OFFSET_CM`) and squares heading.
- **PathTracker:** the `WALL_STOP` → `REANCHOR` → `WALL_DEPART` interlude, one-shot per arrival
  (`_resetNode` latch held through `WALL_DEPART`), gated by the strict-flag check.
- **Config:** `WALL_CONTACT_OFFSET_CM` (axle margin) and `WALL_PIVOT_CLEARANCE_CM` (clearance floor)
  in `config/ConfigNav.h`, the latter hand-mirrored in `validator.py`; a `static_assert` pins
  `WALL_PIVOT_CLEARANCE_CM ≥ WALL_CONTACT_OFFSET_CM` so the depart distance is never negative.

## Trigger to revisit

- The rear-mounted box / manipulator must be pre-staged before a wall press, or the wall-zero
  refused while carrying a piece (variable contact margin) → **issue #77**.
- Real-field wall-zero proves unreliable via encoder-stall → add the deferred hardware path (bumper
  / limit switch, PHASE2_DEFERRED §5.3) through the same interlude.
- The strict wall-normal check needs to be **on by default** → flip
  `Config::WALL_NORMAL_STRICT_ENABLED` (bench-tune `WALL_NORMAL_TOLERANCE_DEG`).
- Cross-axis drift left uncorrected becomes a problem → chain a second reset node on the cross-axis
  wall, or add 2D re-localization.

## See also

- [ADR-001](./001-no-wall-follow-no-dead-reckon.md) — wall-zero is bounded calibration, reconciled above
- [ADR-005](./005-localization-single-source-of-truth.md) — single-writer rule; **amended** here (the new yaw/position writer)
- [ADR-042](./042-operator-offset-localization.md) — the first ADR-005 amendment (operator-offset delta)
- [ADR-027](./027-drive-stop-at-line.md) — the stall watchdog `Drive.wallStop` reuses (meaning inverted)
- [ADR-028](./028-fieldmap-accessor-bounds.md) — `getResetInfo` follows the bounds-checked-sentinel pattern
- [ADR-036](./036-per-turn-pivot-compensation.md) — the sparse-table (`PivotTurn[]`) precedent `RESET_NODES` mirrors
- [issue #77](https://github.com/WinnerWang971119/WRO2026_Code/issues/77) — deferred box pre-stage / carrying-state guard
- [PHASE2_DEFERRED.md §5.3](../PHASE2_DEFERRED.md) — graduated here; the hardware-switch variant remains a future option
