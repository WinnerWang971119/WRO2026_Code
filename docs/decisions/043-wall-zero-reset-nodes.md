# ADR-043: Wall-zero reset nodes — a new node kind that re-anchors pose against a known wall

**Status:** Implemented
**Date decided:** 2026-06 (Phase 2)

## Rule

A `"reset"` node kind carries a parallel **sparse** `RESET_NODES{node, axis, wall_coord, facing}`
table (emitted like the `PivotTurn[]` table of ADR-036, not a per-node field on `NODES[]`).
On arrival at a reset node, PathTracker runs a two-phase interlude:

1. **`WALL_STOP`** — auto-dispatch one latched `Drive.wallStop`: push at low PWM
   (`Config::WALL_STOP_PUSH_PWM`) until the chassis stalls against the wall, then back off
   `Config::WALL_STOP_BACK_OFF_CM`. The stall is detected by **reusing the stop-at-line
   no-progress watchdog** (ADR-027), with the stall meaning **inverted** — here a stall is
   *success* (wall reached), not the `LINE_NOT_FOUND` fault. A push that never stalls within
   `Config::WALL_STOP_TIMEOUT_MS` records a **non-latching** advisory (`Drive.wallStopTimedOut()`);
   it does not halt the FSM.
2. **`REANCHOR`** — one atomic `Localization.reanchorAtWall(axis, wall_coord, facing)` that
   re-zeros the chosen position axis to the wall coordinate **and** squares IMU heading.

An **off-by-default** strict wall-normal flag (`Config::WALL_NORMAL_STRICT_ENABLED == false`)
escalates a bad post-stop heading to `Fault::WALL_NOT_SQUARE`; with the flag off, a skewed stop
is trusted and re-squared anyway.

## Why

Reset nodes kill accumulated encoder/IMU drift against a physical ground truth — the wall —
**without new hardware**: detection is encoder-stall, not a bumper or limit switch. This
reconciles honestly with ADR-001: wall-zero is **bounded calibration**, not continuous
wall-following and not open-ended dead-reckoning. The wall is an absolute reference the robot
touches at a chosen node and then leaves; it *resets* drift to zero rather than tracking against
a surface for navigation — the same family of discrete ground-truth re-anchor as an intersection
crossing, which ADR-001 explicitly endorses.

**Amends ADR-005 (the second amendment).** `reanchorAtWall` is a **new writer** of
`_yawOffsetDeg` and of the position axis state, alongside `zeroHeadingFromCurrent` (heading
offset) and `declarePosition` (position only). It therefore **supersedes ADR-005's "yaw offset
written ONLY by `zeroHeadingFromCurrent`" wording**. The single-**writer** rule is preserved:
only `Localization`'s own methods mutate pose state — the write *set* grows, the writer *module*
does not. ADR-042 was the first amendment (the "never stored" heading clause); this is the second.

## How to apply

- **Authoring:** a `"reset"` node in `field_data`; the pipeline emits a sorted sparse
  `Routes::ResetNode[]` (`N_RESET_NODES` records) read via `FieldMap::getResetInfo(node)`, a
  bounds-checked sentinel accessor (ADR-028). Schema: `{node, axis, wall_coord, facing}`, with
  `axis` encoded **`0 = x`, `1 = y`** (the wall runs parallel to that axis).
- **Drive:** `Drive.wallStop(cm)` — continuous, stall-terminated push; `.setReverse`,
  `.setBackOff`, `.setTimeout` knobs; advisory result via `Drive.wallStopTimedOut()`.
- **Localization:** `reanchorAtWall(uint8_t axis, float wallCoord, float headingDeg)` — atomic;
  re-zeros the axis coordinate and squares heading in one call.
- **PathTracker:** the `WALL_STOP` → `REANCHOR` arrival interlude fires once per reset-node
  arrival (one-shot latch), gated by the strict-flag check before the `reanchorAtWall` write.

## Trigger to revisit

- Real-field wall-zero proves unreliable via encoder-stall (false stalls, or no stall on a
  compliant wall) → add the deferred hardware path: a bumper / limit switch on the leading face
  (PHASE2_DEFERRED §5.3), routed through the same `wallStop` → `reanchorAtWall` interlude.
- The strict wall-normal check needs to be **on by default** (i.e. a skewed stop should fault
  rather than re-square) — flip `Config::WALL_NORMAL_STRICT_ENABLED`.

## See also

- [ADR-001](./001-no-wall-follow-no-dead-reckon.md) — wall-zero is bounded calibration, reconciled above
- [ADR-005](./005-localization-single-source-of-truth.md) — single-writer rule; **amended** here (the new yaw/position writer)
- [ADR-042](./042-operator-offset-localization.md) — the first ADR-005 amendment (operator-offset delta)
- [ADR-027](./027-drive-stop-at-line.md) — the stall watchdog `Drive.wallStop` reuses (meaning inverted)
- [ADR-028](./028-fieldmap-accessor-bounds.md) — `getResetInfo` follows the bounds-checked-sentinel pattern
- [ADR-036](./036-per-turn-pivot-compensation.md) — the sparse-table (`PivotTurn[]`) precedent `RESET_NODES` mirrors
- [PHASE2_DEFERRED.md §5.3](../PHASE2_DEFERRED.md) — graduated here; the hardware-switch variant remains a future option
