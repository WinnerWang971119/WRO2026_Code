# ADR-031: CTPS ingests a derived mission trace recorded from the real C++

**Status:** Implemented
**Date decided:** 2026-06 (issue #33 CTPS)

## Rule

The Clear Trajectory Prediction Simulator (CTPS) is fed a **derived mission
trace** produced by a host build of the real production C++ (ingestion
Mechanism #4), not a Python re-implementation of mission or task bodies. A
`CTPS_RECORD`-guarded hook in `Mission.cpp` records the mission-level command
stream (`goto` / `wait` / `print` / `beep` / `led` / `task_start` with args /
`task_end` / `build_error`); recording facade mocks under `ctps_mocks/` capture
the `manip` / `drive` calls made inside task bodies. The C++ `ctps_dump` emits
that command stream as JSON, a derived artifact in the same sense as `routes.h`.
A Python package (`tools/path_planner/ctps/`) expands every `goto` through the
parity-locked `PathTrackerSim` and renders a self-contained HTML/Canvas player.
There is no Python mirror of mission or task bodies.

## Why

Recording at the hardware-facade boundary is robust to helpers, branches, and
loops inside a task body. Hand-mirroring the bodies in Python (the ADR-025
sub-decision this supersedes) and regex-scraping the source both drift the
moment a body grows control flow; recording the calls the real code actually
makes does not. The host hook in `Mission.cpp` is a small, `#ifdef`-guarded
deviation from a strict "mocks-only, Mission untouched" approach. It is needed
because Mission's own `wait`, the `task` name and args, and `build_error` are
not observable at the facade boundary. The firmware build is byte-unchanged: the
macro is undefined for the Arduino target, so the hook compiles out entirely.
The player is a self-contained HTML/Canvas file so a recorded run scrubs in O(1)
with no server or toolchain at field-day.

Out of scope, stated explicitly: CTPS does not model the fault FSM (beep / flash
/ await-reset) and does not aim for physical fidelity. Playback is
constant-velocity for smooth scrubbing, not a real-duration prediction. The
drift flag is the only physical problem CTPS surfaces.

## How to apply

- Recording facade mocks live in `WRO2026/test/ctps_mocks/`; they capture the
  `manip` / `drive` calls a task body makes.
- The mission-level command stream is captured by the `CTPS_RECORD` hook in
  `WRO2026/Mission.cpp`. Keep the hook `#ifdef CTPS_RECORD`-guarded so the
  firmware build stays byte-identical.
- `ctps_dump` emits the recorded stream as the derived JSON trace.
- The Python side reads, expands, and renders via
  `tools/path_planner/ctps/{trace_loader, track_builder, player_html}`; `goto`
  expansion goes through the parity-locked `PathTrackerSim`.

## Trigger to revisit

- A logged-trace adapter for the LRS replaces or augments the recorded trace as
  the ingestion source.
- Real `Manipulator` / `Drive` tasks land and need richer recording than the
  current facade mocks capture.
- A need for timing or physics fidelity arrives, at which point constant-velocity
  playback and the fault-FSM exclusion must be reopened.

## See also

- [ADR-025 — Tasks layer composes subsystem calls as blocking method bodies](./025-tasks-as-blocking-methods.md) — supersedes its "Python mirror of each task body" sim-parity sub-decision.
- [ADR-022 — field_data geometry lives in JSON; field_data.py is a loader](./022-field-data-json-loader.md) — the derived-artifact precedent (`routes.h`).
- [ADR-030 — Host-compile harness links the real production symbols, CI-enforced](./030-harness-links-real-sources-ci-enforced.md) — the host harness CTPS records through.
