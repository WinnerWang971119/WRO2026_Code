# ADR-051: CTPS mock-builder surface kept a superset of the real builders by two complementary guards; mock records at an `_executed`-guarded destructor

**Status:** Implemented
**Date decided:** 2026-07 (Phase 3 / testing, issue #89)

## Rule

The CTPS mock builder surface under `WRO2026/test/ctps_mocks/` is kept a **superset**
of the real `Drive`/`LineFollower` fluent-builder surface by two complementary guards,
and every mock builder records in an **`_executed`-guarded destructor** mirroring the
real builder pattern:

1. **Compile-time conformance TU** (`ctps_mock_conformance` ctest target) — a host TU
   that chains every *current* real setter (with its real argument type) on each mock
   builder as a discarded temporary. It catches a removed or renamed mock setter and a
   signature-incompatible existing setter. Known limits: it does NOT auto-catch a real
   builder *gaining* a new setter (the TU stays green until a human adds the call); it
   verifies call-site assignability, not exact parameter types (a convertible type can
   pass). Always-built; a missing or mismatched setter breaks the build.

2. **Python name-superset check** (`test_ctps_mock_superset.py`) — asserts that each
   mock builder's public fluent-setter name-set is ⊇ its real counterpart's by parsing
   both headers. Catches the "real builder grew a setter the mock lacks" case — the
   actual #89 drift class the compile TU cannot auto-catch. Includes a negative
   synthetic-mock unit test to prove the guard bites.

Together they deliver "can't drift again" across both drift directions.

**Destructor recording seam:** each mock builder stores the fields from every chained
setter and emits exactly once in the destructor, guarded by `_executed` (mirrors the
real builders at `Drive.h:1499–1521`, `:1752–1758`). Copy and move constructors are
deleted; a `static_assert(!is_copy_constructible && !is_move_constructible)` documents
the invariant. The mock stays non-movable while the real builders are movable — CTPS
only ever sees discarded temporaries, C++17 guaranteed elision means no move is
invoked, and the `_executed` guard makes emission idempotent if that ever changes.

## Why

The `flat()`/`setPreset()` gap (#89) arose because the mock was never updated when the
real builders grew those setters — no name-checker existed in the repo. Updating the
mock without adding a guard reproduces the same failure mode on the next setter
addition. The two guards close both drift directions: the compile TU catches a drift
toward the mock *losing* an existing setter; the Python name-superset check catches a
drift toward the real *gaining* a setter the mock lacks.

The destructor seam is chosen over record-at-construction because chained setters like
`.setMode(GYRO).setHeading(0).setReverse(true)` are not yet visible at construction
time; the destructor fires after the full chain has settled. The `_executed` guard
keeps emission idempotent.

## How to apply

- **Adding a real setter to `Drive` or `LineFollower` builders:** add the matching
  member + setter to the corresponding mock builder, then add the call to
  `test_ctps_mock_conformance.cpp`. The Python name-superset check catches the
  omission if that human step is missed.
- **Adding a new mock builder:** delete copy + move ctors; add
  `static_assert(!is_copy_constructible && !is_move_constructible)`; emit via
  `ctps::recordDriveMove` in an `_executed`-guarded destructor; add a conformance
  probe for every setter to the TU.
- **The TU target** lists `test_ctps_mock_conformance.cpp` + `ctps_record.cpp` +
  `routes_fixture.cpp` as its `add_executable` sources (the first supplies the mock
  facade globals + `recordDriveMove`, the routes TU the graph symbols the mock headers
  pull in), and force-includes the mock + fixture headers via
  `add_ctps_mock_force_includes` + `add_fixture_routes_force_include` (the fixture-routes
  one first, so its `ROUTES_H` wins). Give the TU a unique filename with its own
  `main()` so it is never globbed or force-included into another target.

## Additive: per-command speed knobs + `recordDeclarePosition` (2026-07, issue #130)

`DriveMoveParams` grew `power`/`preset`/`startSpeed`/`endSpeed`/`profiled`/`brake`
fields (the `.setPower`/`.setPreset`/`.setStartSpeed`/`.setEndSpeed`/`.tShape`/
`.flat`/`.setBrake` knobs the mocks previously dropped), captured at all five
construction sites — the three `Drive.h` builders plus both `LineFollower.h`
`lineFollowDist` sites — and a new `recordDeclarePosition(name)` free function
hooked into `Mission.cpp::declarePosition`'s success path. Both guards above
extend to the new fields/setters unchanged: the conformance TU chains each new
setter, and the Python name-superset check covers them. No new drift class —
same two-guard mechanism, more surface. See ADR-058 for why (firmware-faithful
CTPS playback needs the resolved per-command speed).

## Trigger to revisit

- If a real setter uses a narrower type than call-site assignability accepts (e.g., a
  distinct enum vs. a plain integer), add an exact-type `static_assert` probe to the
  TU for that setter.
- If the header-parsing heuristic in `test_ctps_mock_superset.py` cannot handle a new
  builder file layout, update the parser or switch to a libclang-based extractor.

## See also

- [ADR-030 — Host-compile harness links the real production symbols; CI-enforced](./030-harness-links-real-sources-ci-enforced.md) — the `ctps_mock_conformance` target is an always-built CI-enforced ctest that extends this rule to the CTPS mock surface.
- [ADR-049 — Host-test mock HAL: thin record-and-test-set fidelity, dedicated-dir shadowing rule](./049-host-test-mock-hal.md) — the trust-boundary rule and `dedicated-dir / force-include` pattern that `ctps_mocks/` follows.
- [ADR-033 — CTPS ingests a derived mission trace; no Python re-implementation of task bodies](./033-ctps-derived-mission-trace.md) — the CTPS recording facade and `drive.method` 3-place contract the mock builders feed.
- [ADR-058 — CTPS playback renders the firmware's T-shape motion profile](./058-ctps-firmware-motion-profile.md) — consumer of the per-command speed knobs this guard was extended to cover.
- Issue #89 (the `flat()`/`setPreset()` drift that motivated these guards).
