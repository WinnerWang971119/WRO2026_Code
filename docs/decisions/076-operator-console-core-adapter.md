# ADR-076: Operator diagnostics console — `src/operator/` domain, OLED-free core/adapter split

**Status:** Implemented
**Date decided:** 2026-07 (Phase 6)

## Rule

The operator diagnostics console (#166) lives in a new top-layer domain, `src/operator/`,
split into an OLED-free core (`OperatorConsole`) and a thin Arduino adapter
(`OperatorPanel`). The core owns the item table, the scroll/confirm FSM, button-edge
detection, every live view, and the SELF TEST sequencer; it is pure — no `<Arduino.h>`,
no `millis()`, no I/O, time only from a pushed-in `nowMs`. The adapter samples hardware,
renders `ConsoleFrame` to the OLED, drives LEDs/buzzer, and executes the core's
`ActionRequest`s. Two rules follow from the same split: **views are per-tick FSM states,
actions are sanctioned blocking dispatches**; and **the adapter pushes cheap cached state
on a cadence while the core pulls every expensive or latching read** (`READ_MOSAIC`,
`READ_LINE_BOARDS`) via `ActionRequest`. Line-board liveness is proven only by
`WROLineArray::probeBoard()`'s I²C address ACK, never by channel values.

## Why

`src/util/` is the *bottom* layer — `WROMath`, `TickProfiler`, `EncoderSource` all sit
below `Drive`. A console that samples `WROLineArray`, `Vision` and `YawSource`, commands
`Drive` (command-only, via the `CAL_MOTOR` dispatch), and surfaces `Localization`'s
`begin()` result as a `BootReport` flag is a top-layer peer of `Mission`; filing it under
`util/` would invert the dependency arrow the rest of the codebase enforces.

The core/adapter split is load-bearing, not stylistic. `WRO2026/test/CMakeLists.txt:360-364`
documents that no real `EventTrace.cpp` is host-linked — the harness force-includes a
no-op mock instead — specifically to dodge the `MiniR4OLED.h` `WHITE`-macro collision. Any
OLED-owning `.cpp` is excluded from the host harness by that same mechanism, so a
monolithic console `.cpp` moved out of `WRO2026.ino` would inherit the exclusion and buy
zero CI coverage. Splitting the item table, scroll FSM, views, and SELF TEST sequencer
into an OLED-free core is what makes any of it host-testable.

A no-timeout view is deliberately per-tick, not a blocking spin: an unbounded operator
hold is not what iron-rule-1's sanctioned-blocking-sites escape hatch (`_drainBlocking()`,
`calibrateMotorBias()`, `WROLineArray::calibrate()`, `readMosaic()`) exists for. Those are
bounded, dispatched actions; the console's own dispatched actions reuse that category, but
its views — which have no exit deadline — must not.

Pure adapter-push would delete the only mechanism that can order a fresh read before
SELF TEST judges a check, and would fire the ~240 ms blocking `readMosaic()` on a
free-running cadence whether or not anything needs it. Pure core-pull would force
pointless round-trips for state that is already cheaply cached. The hybrid gives each
`DiagSample` field exactly one initiator and one authoritative channel.

Finally, `WROLineArray::_readBoardInternal()` states outright that
`MatrixColor::getColor(ColorType)` has no error return, so a dead I²C bus reads as a `0`
byte indistinguishable from a legitimately dark channel — channel values can never prove a
board answered. `probeBoard()` sidesteps that by checking only for the mux-selected
address ACK.

## How to apply

- `src/operator/ConsoleTypes.h` (PODs, `<stdint.h>`-only), `OperatorConsole.h/.cpp` (core,
  host-tested), `OperatorPanel.h/.cpp` (adapter, not host-linked, same as `EventTrace.cpp`).
- `OperatorConsole` never includes `Config.h` (it pulls `<Arduino.h>`) — only
  `config/ConfigUi.h` + `config/ConfigGlobal.h` directly, the precedent `Vision.h` set.
- `OperatorPanel::update()` is the sole caller of `readBoard()`/`probeBoard()`/
  `readMosaic()` on the console's behalf, and only as execution of a core
  `ActionRequest` — never on the adapter's own push cadence.
- `EventTrace::setRenderEnabled(false)` (this same change) gives the console exclusive
  OLED ownership without silencing `EventTrace`'s Serial heartbeat or its polling.

## Trigger to revisit

If a future console entry needs to command hardware for longer than one tick can bound
(anything beyond the two pre-existing calibration dispatches), reconsider whether it
belongs behind a per-task FSM (ADR-025's Phase 6 note) rather than a console
`ActionRequest`.

## See also

- [ADR-049](./049-host-test-mock-hal.md) — the shared mock-HAL trust boundary this
  console's host-harness exclusion follows the same shape as.
- [ADR-003](./003-arduino-style-modules.md) — the no-`delay()`-in-`update()` rule this
  ADR's views-are-FSM-states clause restates for the console specifically.
- `WRO2026/test/CMakeLists.txt:360-364`, `WROLineArray::_readBoardInternal()` in
  `WRO2026/src/perception/WROLineArray.cpp` — the no-error-return note the liveness rule rests on.
- `docs/plans/2026-07-20-operator-diagnostics-console.md` — the originating plan.
- Issue #166.
