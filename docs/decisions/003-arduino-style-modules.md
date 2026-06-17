# ADR-003: Arduino-style modules — no class hierarchies, no DI, fixed-size arrays, no `delay()` in `update()`

**Status:** Implemented
**Date decided:** 2026-05 (Phase 0)

## Rule

- Modules are paired `.h`/`.cpp` files (or header-only) — not class hierarchies.
- Global singletons exposed via `extern` — not dependency injection.
- Fixed-size arrays — no `std::vector` / `std::map` / `new` / `malloc` in steady state.
- Every long-running module exposes a non-blocking `update()`; no `delay()` inside any `update()` path. Time is tracked with `millis()`.

## Why

The Matrix R4 runs C++ on a microcontroller with limited RAM, no operating system, and a single-threaded `setup()`/`loop()` execution model. Arduino code that tries to look like a Python service ends up either failing to compile, fragmenting memory, or blocking the main loop in ways that break real-time response. This is not stylistic — on Arduino, the alternatives literally do not work.

## How to apply

- New module = `Foo.h` + `Foo.cpp` + `extern FooClass Foo` + `FooClass Foo;` in the .cpp.
- All state lives in the singleton instance or in module-private static.
- Long operations decompose into state machines driven by `update()`.
- Only `delay()` / blocking sites permitted: command-time orchestration (`Drive::_drainBlocking`, `Drive::calibrateMotorBias`, `WROLineArray::calibrate`, `Vision::readMosaic`) — never inside a cooperative tick. Each blocking site must spin the registered `cooperativeTick` callback so OLED/heartbeat/sensors keep running during the wait.

## Trigger to revisit

A platform change (e.g., RTOS-capable controller) — until then, this is fixed.

## See also

- [ARCHITECTURE.md](../../ARCHITECTURE.md) — the two iron rules
