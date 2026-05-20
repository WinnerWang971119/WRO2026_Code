# ADR-010: Lightweight in-memory logging only — no SD, no replay tooling

**Status:** Implemented (superseded the original "no logging in scope" decision once EventTrace shipped)
**Date decided:** 2026-05 (Phase 0); EventTrace shipped 2026-05 (Phase 1)

## Rule

Debug visibility is limited to:
- A 4-entry OLED ring buffer of recent events (`EventTrace`).
- A 500 ms Serial heartbeat (`HB t=... yaw=... node=...`).
- Edge-triggered Serial event lines on faults / state transitions.

**Not in scope:** SD-card logging, persistent log files, replay/record tooling, post-hoc data analysis pipelines.

## Why

The upfront cost of doing logging "correctly" — making every module's inputs deterministic and recordable — is high enough to slow down the more important work of getting basic navigation working. EventTrace gives enough visibility for in-the-loop debugging (OLED + Serial console) without the per-module cost of full instrumentation. If something goes wrong in competition, the team observes the OLED trace; they don't go to post-hoc data analysis.

## How to apply

- Any module can call `EventTrace.pushEvent("MSG")` from `setup()` or from a fault transition. Up to 20 chars per line. Wired into `cooperativeTick()` so transitions render within one loop iteration.
- EventTrace is **read-only** with respect to other modules — it polls public state and does not influence behavior.
- Serial heartbeat period is `Config::MISSION_HEARTBEAT_MS = 500`.

## Trigger to revisit

A bug that's only reproducible at the venue, and the OLED trace + heartbeat aren't enough to root-cause it. Cost of SD logging then becomes worth paying.

## Supersedes

The original decision (Phase 0 draft of `DECISIONS.md` "What is deliberately not in scope right now") put *all* logging out of scope. EventTrace landed in Phase 1 as a small in-scope exception; this ADR reflects that.

## See also

- `WRO2026/EventTrace.h` — implementation
- [ARCHITECTURE.md](../../ARCHITECTURE.md) — Out of scope section
