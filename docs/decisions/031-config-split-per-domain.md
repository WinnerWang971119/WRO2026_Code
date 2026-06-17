# ADR-031: Config split per domain — umbrella Config.h + per-domain sub-headers + ConfigChecks.h last

**Status:** Implemented (Refined by ADR-039)
**Date decided:** 2026-06 (Phase 5 pulled forward)

## Rule

The monolithic `Config.h` is replaced by a thin umbrella header that `#include`s per-domain
sub-headers in logical order, then `ConfigChecks.h` as the final include. All consumers continue
to include `Config.h` only — never a sub-header directly.

Current split:
- `ConfigManipulator.h` — servo angles, belt positions, timing constants for the manipulator.
- `ConfigChecks.h` — all validation `static_assert`s (must be last; it checks symbols from all sub-headers).

Existing drivetrain, line-follower, and motor-ID constants and their inline `static_assert`s
remain in `Config.h` itself. They are **not** relocated by this ADR.

## Why

The flat `Config.h` exceeded 990 lines. At competition, a pit-tuner searching for a servo
angle wades through motor-PID gains, junction debounce timers, and encoder PPR tables to find
one constant. Per-domain sub-headers make comp-day tuning fast by co-locating only the constants
that move together.

Placing `ConfigChecks.h` last guarantees every `static_assert` sees every symbol regardless of
the order sub-headers are included. Keeping the split minimal (only adding `ConfigManipulator.h`
now) avoids the high risk of accidentally relocating the load-bearing motor-ID asserts that
Drive.cpp depends on at link time.

This supersedes the earlier informal convention that `Config.h` is a non-negotiable single file.

## How to apply

- **New subsystem** → create `ConfigXxx.h`, write constants there, add one `#include "ConfigXxx.h"`
  in `Config.h` before the `#include "ConfigChecks.h"` line.
- **New invariant** → add a `static_assert` to `ConfigChecks.h`. Never add a new `static_assert`
  to a sub-header; only the checks file has the full symbol set.
- `ConfigChecks.h` guards itself with `#ifndef CONFIG_H` / `#error "Include Config.h, not ConfigChecks.h directly"` to catch accidental direct includes.
- All existing code continues to `#include "Config.h"` — no call-site changes needed.

## Trigger to revisit

- An include-order bug causes a `static_assert` in `ConfigChecks.h` to fire against an
  uninitialized value, meaning the umbrella order is wrong.
- The split needs to extend to drivetrain or line-follower constants — the motor-ID asserts
  must travel with the constants they check; plan the relocation carefully.

## See also

- [ADR-022](./022-dual-motor-profile.md) — dual motor profile; its constants currently live in the
  un-split `Config.h` and are not moved by this ADR.
- [ADR-039](./039-config-split-finished.md) — finishes the split: remaining constants into `config/`, all asserts centralized in `ConfigChecks.h`.
- `WRO2026/Config.h`, `WRO2026/ConfigManipulator.h`, `WRO2026/ConfigChecks.h`
