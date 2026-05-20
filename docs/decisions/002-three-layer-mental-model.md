# ADR-002: Three-layer architecture — perception → map → planning

**Status:** Implemented
**Date decided:** 2026-05 (Phase 0)

## Rule

The system is organized into three layers, each consuming the abstraction of the layer below:
1. **Perception** — raw sensor readings → discrete events ("centered on line", "at intersection").
2. **Map** — hand-built graph: nodes are intersections, edges are line segments tagged with cardinal directions.
3. **Planning** — Dijkstra over the graph → node sequences → low-level commands keyed off current heading.

Task logic never touches motors directly; it composes navigation primitives.

## Why

This separation is what makes the system feel programmable instead of scripted. A new task is a few lines of orchestration on top of existing primitives, not a new motion sequence. Without this separation, every new task duplicates motion logic and the codebase silently forks.

## How to apply

- Mission DSL verbs (`goTo`, `wait`) call `PathTracker` and `Drive` — never the vendor library directly.
- New tasks must compose existing primitives; if a task wants a new primitive, the primitive belongs in `Drive` / `LineFollower` / `Manipulator`, not in the task.
- The map layer (`FieldMap` + `routes.h/cpp`) is the only place that knows the field geometry.

## Trigger to revisit

If a task ever needs raw motor commands or raw sensor reads in its own code, that's a sign the layer below it is missing a primitive — add the primitive instead of breaking the layering.

## See also

- [ARCHITECTURE.md](../../ARCHITECTURE.md) — dependency graph enforcing top-down calls
- [ADR-003](./003-arduino-style-modules.md)
