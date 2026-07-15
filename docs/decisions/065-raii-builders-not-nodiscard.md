# ADR-065: RAII auto-execute builders are not `[[nodiscard]]`

**Status:** Implemented
**Date decided:** 2026-07 (Phase 6)

## Rule

Fluent RAII auto-execute builders (ADR-012) and the verbs that return them carry no
`[[nodiscard]]` attribute — neither on the class nor on the verb declaration. A bare verb
statement (`Manipulator.grab();`, `LineFollower.setEnabled(false);`, `Drive.driveDist(30);`)
is a canonical call shape and must compile warning-free. Never bind a builder to a named
variable: that defers the destructor's apply/drain to end of scope.

## Why

ADR-012 marked every builder `[[nodiscard]]` on the belief that it turns "built it and forgot
to execute" into a compile warning. It does the opposite. `[[nodiscard]]` fires when a call's
return value is *discarded* — exactly the bare-statement shape above, where the discarded
temporary still executes via its destructor. The real hazard — binding the builder to a local
(`auto b = Drive.driveDist(30);`), which defers execution to end of scope and can corrupt
command ordering — *binds* the value rather than discarding it, so `[[nodiscard]]` never
fires on it. The attribute therefore produced only false positives: 21 warnings on correct
call sites in the host build (surfaced by AppleClang as `-Wunused-value`), papered over by
per-compiler suppressions (`/wd4834`, `-Wno-unused-result -Wno-attributes`) that also masked
genuine diagnostics. Chained calls dodged the warning only by accident: setters return
references, which class-level `[[nodiscard]]` does not propagate through.

## How to apply

- No `[[nodiscard]]` on builder classes or builder-returning verbs: Drive's six builders,
  `LineFollowerEnableBuilder`, `PathTrackerGoToBuilder`, `ManipAction` — and their host-test
  mocks, which mirror the production surface.
- The host harness (`WRO2026/test/CMakeLists.txt`) carries no discarded-value suppressions
  (`/wd4834`, `-Wno-unused-result`, `-Wno-attributes` are removed and must stay out).
- `[[nodiscard]]` stays appropriate for pure queries whose returned value is the entire
  point of the call; this ADR exempts only RAII auto-execute builders.

## Trigger to revisit

A toolchain or lint rule that can diagnose "builder bound to a named variable" — the actual
hazard. If one is adopted, wire it in place of `[[nodiscard]]`.

## See also

- [ADR-012](./012-fluent-builder-3plus-knobs.md) — the builder pattern itself; its
  `[[nodiscard]]` clause is superseded by this ADR
- [ADR-016](./016-sync-default-builders.md) — destructor blocking policy
- [ADR-032](./032-manipulator-api-pattern.md) — Manipulator verbs follow the same shape
