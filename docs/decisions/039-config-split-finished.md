# ADR-039: Finish the per-domain Config split (config/ folder + centralized checks)

**Status:** Implemented
**Date decided:** 2026-06

## Rule

The remaining `Config.h` body is split into per-domain sub-headers under
`WRO2026/config/`: `ConfigGlobal.h`, `ConfigDrive.h`, `ConfigLine.h`,
`ConfigNav.h`, and `ConfigMission.h`, joining the already-split
`ConfigManipulator.h`, `ConfigVision.h`, and `ConfigChecks.h` which are also
relocated into that folder. Root `Config.h` becomes a pure umbrella: it
`#include`s all sub-headers in a load-bearing order and ends with
`#include "config/ConfigChecks.h"` last.

Every `static_assert` in the config layer now lives in `ConfigChecks.h`. This
**reverses** the prior "tripwire inline next to its constant" convention from
ADR-031: the explanatory block comment and `@warning` Doxygen tag stay beside
the constant, but the `static_assert` statement itself is centralized.

## Why

ADR-031 carved out only `ConfigManipulator.h` and `ConfigVision.h` — the
drivetrain, line-follower, nav, and mission constants remained in the flat root
`Config.h`. This ADR finishes what ADR-031 started. Per-module buckets let a
pit-tuner find a constant fast on comp day without wading through unrelated
domains. Centralizing all asserts in `ConfigChecks.h` lets them cross-reference
every domain symbol from one place, which the umbrella enables by pulling Checks
in last.

The include order inside the umbrella is load-bearing because the sub-headers
forward each other's `constexpr` values: `ConfigGlobal.h` defines
`PWM_MIN_MAGNITUDE`, which `ConfigDrive.h` consumes via
`TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE` — so Global must precede Drive.
`ConfigDrive.h` in turn defines `SPEED_NORMAL`, which `ConfigLine.h` consumes
via `CRUISE_SPEED = SPEED_NORMAL` — so Drive must precede Line. That gives the
chain `ConfigGlobal → ConfigDrive → ConfigLine`, which is exactly the umbrella
order, and each consumer also `#include`s the header it depends on.

## How to apply

- **New constant** → add it to the relevant `config/Config*.h` (never to the
  root `Config.h` umbrella).
- **New invariant** → add a `static_assert` to `config/ConfigChecks.h`. Never
  put a `static_assert` inline in a sub-header; `ConfigChecks.h` is the only
  place that sees all symbols simultaneously.
- **Consumers** keep `#include "Config.h"` — no call-site changes needed.
- The four direct sub-header includers (`ManipServo.h`, `ManipBelt.h`, `Vision.h`, `VisionFrame.h`) update their includes to `#include "config/Config*.h"`.

## Trigger to revisit

- An assert failure in `ConfigChecks.h` is hard to trace back to its constant
  because the explanatory comment and the `static_assert` are now in separate
  files — if this hampers debugging, move the assert back inline.
- A sub-header grows large enough to warrant its own further split.

## See also

- [ADR-031](./031-config-split-per-domain.md) — original split decision; this ADR finishes it
- Issue #39 — Config split tracking issue
- Issue #66 — deferred sketch-reorg (moving `Config.h` umbrella to `config/` top-level)
- `WRO2026/config/` — the sub-header folder this ADR establishes
