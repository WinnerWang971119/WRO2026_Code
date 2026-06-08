# ADR-007: Manipulator exposes control primitives only — no high-level action verbs until the mechanism is built

**Status:** Superseded by ADR-032
**Date decided:** 2026-05 (Phase 0)

## Rule

`Manipulator` exposes only low-level control: `setLift(h)`, `setGripper(angle)`, `setArm(angle)`, `update()`, `isDone()`. No `pickTool()` / `placeTile()` / `dropCement()` until the mechanism is built and tested.

## Why

The mechanical design for picking up tools, placing tiles, and handling cement blocks is not yet finalized, and the three object types differ significantly in shape and weight. Committing to high-level action names now would invite redesign after the mechanism is built. This is a deliberate "decide as late as possible" choice for the part of the system whose hardware is least settled.

## How to apply

- `Manipulator.h/.cpp` currently stub every method (returns true / no-op). Real implementation lands once mechanism is built.
- High-level actions (`pickTool`, `placeTile`, etc.) live in `Tasks`-style state machines, composed from the three primitives, once the primitives are real.

## Trigger to revisit

Manipulator mechanism v0 lands (Phase 3 entry). At that point the primitive parameter ranges and units get locked, and the first composed actions can be written.

## See also

- [Roadmap.md](../../Roadmap.md) — Phase 3 manipulator
- [ADR-032](./032-manipulator-api-pattern.md) — superseding decision: verb-level builder API now that the CAD is done
