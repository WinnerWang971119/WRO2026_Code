# ADR-005: `Localization` is the single source of truth for current node + heading

**Status:** Implemented
**Date decided:** 2026-05 (Phase 0)

## Rule

Exactly one module is allowed to answer "where am I?" — `Localization`. It holds the current node and the IMU-yaw offset. Every other module reads from it; only the navigation primitives (`PathTracker` via `onIntersection` / `onDeadReckonedAdvance` hooks) write to it.

No other module may keep a local copy of the position.

## Why

As the codebase grows, the temptation to "just keep a local copy" of the position becomes enormous — and that's exactly how silent inconsistencies enter a system. With one writer, there is exactly one place to look and exactly one place to fix when position state goes wrong.

## How to apply

- `Localization.getCurrentNode()` / `getHeadingDeg()` — reads, allowed anywhere.
- `Localization.onIntersection(advanceTo)` / `onDeadReckonedAdvance(advanceTo)` — writes, called only from `PathTracker`.
- `Localization.zeroHeadingFromCurrent()` — called once at mission arm (BTN_UP press in the sketch).
- Heading is computed live from IMU yaw + offset, never stored — no cache means no staleness.

## Trigger to revisit

If a perception module (camera, color sensor) ever needs to *correct* localization based on what it sees (e.g., visual SLAM-style anchoring), the write set may need to grow — but it should still go through a single `Localization` method, not a direct write.

## See also

- [ADR-002](./002-three-layer-mental-model.md)
- [ARCHITECTURE.md](../../ARCHITECTURE.md) — coding criteria checklist
