# ADR-049: Host-test mock HAL as the shared trust boundary

**Status:** Implemented
**Date decided:** 2026-06 (Phase 3 / testing, issue #105)

## Rule

One shared mock HAL under `WRO2026/test/hal_mocks/` is the trust boundary for
host-linking production `.cpp` against a headless host. Mocks are **thin
record-and-test-set**: they record call arguments, expose plain test-settable
state, and faithfully reproduce single-call vendor contracts (e.g.
`resetCounter()` zeroes the encoder backing; an error read leaves out-params
untouched) — but they never model multi-tick FSM or done-bit evolution. Each
mock header carries a "Fidelity — where this mock can lie" docblock.

Per the `mocks/`-shadowing rule (see gotcha memory), a facade mock that not
every target needs lives in its own dedicated dir (`hal_mocks/`, the same
pattern as `ctps_mocks/`), force-included into the one target that needs it;
`mocks/` is never added to that target's include path (ODR mutual-exclusion
invariant). Each mock reuses the production include guard (`MATRIXMINIR4_H`,
`DRIVE_H`, `EVENTTRACE_H`) so a force-include ahead of the production header
makes the production header no-op.

After the `hal_mocks/` surface is proven green, `mocks/MatrixMiniR4.h` is
collapsed to a thin forwarding shim over `hal_mocks/MatrixMiniR4.h` so there
is a single facade definition; `ctps_mocks/` is left untouched.

## Why

The host harness previously re-typed production logic inline as `// MIRROR:`
slices (epic #107). Those mirrors drift silently: the replica freezes behavior
at copy time, so a real regression in the production symbol passes undetected
— precisely the disease ADR-030 was written to prevent. One faithful HAL lets
real `.cpp` host-link and the mirrors be deleted (#106, #46).

Behavioral modeling of the STM32 done-bit handshake was evaluated and rejected
via an adversarial debate (Strong No, 3.5/10). The done-bit is a coprocessor
signal the host cannot observe; any invented timing drifts into false-green
test results — the same disease a `// MIRROR:` slice produces. A mock that
re-implements the completion FSM is just another mirror. Single-call vendor
contracts ARE modeled because they are deterministic observations, not logic,
and the existing mocks already follow that pattern.

## How to apply

- **New host target that links a hardware-touching `.cpp`** → force-include
  the relevant `hal_mocks/` headers (NOT in shared `mocks/`, which shadows
  every target); wire via `add_hal_force_includes(target)` (MSVC `/FI` +
  `SHELL:-include` for g++); keep `mocks/` off the target's include path.
- **Mock fidelity:** record call args + expose test-settable state; document
  every lie in the per-header "Fidelity" docblock; do not add multi-tick
  behavioral modeling, even when the real SUT polls a done-bit.
- **ODR rule:** no single CMake target may have both `mocks/` and `hal_mocks/`
  reachable. The `DRIVE_H` / `MATRIXMINIR4_H` guards are shared; a double-
  reachable path produces two conflicting class definitions.
- **Conformance proof:** the `test_mock_hal` target includes a never-run
  `if (false) { ... }` block that compiles every real `Drive.` / `MiniR4.` /
  `EventTrace.` call expression grepped from the production `.cpp` files
  (`Drive.cpp`, `Mission.cpp`, `PathTracker.cpp`). A missing or wrong-signature
  mock method fails the build. The grep-derived call list is captured in the PR
  body for reuse as the #106 link checklist.

## Trigger to revisit

- When #46 / #106 host-link real `Mission.cpp` / `PathTracker.cpp` /
  `Drive.cpp` against this HAL — the OLED mock currently omits the vendor
  `WHITE` macro that `EventTrace.cpp` uses (`MiniR4.OLED.setTextColor(WHITE)`);
  add a guarded `#define WHITE 1` (push_macro / undef / pop_macro) when real
  `EventTrace.cpp` links. (ADR-related gotcha: `#define WHITE` collides with
  `enum class { WHITE }`.)
- If a mock's recorded surface proves insufficient for a real consumer's logic,
  extend the recorded surface — but never add multi-tick behavioral modeling.
- If the `mocks/MatrixMiniR4.h` forwarding shim (T7 consolidation) breaks an
  existing ctest green gate, revert the shim and leave the dual-facade as a
  documented fast-follow; the `hal_mocks/` dir stands alone.

## See also

- [ADR-030 — Host-compile harness links the real production symbols; CI-enforced](./030-harness-links-real-sources-ci-enforced.md) — the parent rule this HAL serves.
- [ADR-033 — CTPS recording facade mocks under `ctps_mocks/`](./033-ctps-derived-mission-trace.md) — the dedicated-dir precedent `hal_mocks/` follows.
- `docs/plans/2026-06-25-host-mock-hal.md` — originating plan (T9).
- Issues #105 (this HAL), #107 (mirror-slice epidemic), #106 (PathTracker/Drive real link), #46 (Mission real link).
- Gotcha memories: `gotcha_mocks_dir_shadows_all_targets`, `gotcha-drive-not-host-compiled`, `gotcha-oled-white-macro-vs-enum`.
