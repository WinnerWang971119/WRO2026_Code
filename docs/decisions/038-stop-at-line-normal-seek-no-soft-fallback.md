# ADR-038: Blind→projection auto stop-at-line fires a normal seek — no soft-fallback

**Status:** Implemented
**Date decided:** 2026-06 (Phase 3)

## Rule

The blind→projection auto-seek (ADR-037) fires a **normal** `Drive.stopAtLine`: it
runs to the default margin (`expected + Config::STOP_AT_LINE_MAX_MARGIN_CM`), with no
zero-margin `setMaxDistance(distance_cm)` override. A missed line is a **hard fault** —
`Drive` latches the no-line fault and the shared guard at the top of
`PathTracker::update()` enters `Fault::DRIVE_FAULT`, identical to every other
`stopAtLine`. There is no degrade-to-dead-reckon recovery.

The `softFallback` resolver output, the firmware `_activeSoftFallback` latch + its
recovery branch, and the `Config::STOP_AT_LINE_FAULT_FALL_BACK` toggle are all deleted.
`resolveStopAtLine` returns a clean `{seek, motionMode}` again. The auto-FIRE itself
(the `Routes::BLIND_PROJECTION_AUTO_SEEK` toggle, the per-edge `stop_at_line_override`,
and the resolver's seek/motion precedence ladder) is unchanged — only the *soft*
behaviour on a miss is removed.

## Why

A `softFallback` output with no live consumer is exactly the speculative abstraction the
codebase bans. It existed to flag "this seek may degrade", but the degrade path it fed
was gated off in development and carried two latent bugs (a fault-only recovery that
never ran the overshoot roll, and a corrective advance whose geometry was never
bench-verified). Deleting the flag removes both bugs by construction and restores the
two-output `(seek, motionMode)` resolver that both languages and the truth-table CSV
transcribe.

Fail-loud is the right posture for both development and competition: a missed anchor
line means the chassis is mis-localized, and silently dead-reckoning past it compounds
the error. Surfacing it as a hard `DRIVE_FAULT` is the same loud signal every other
`stopAtLine` already gives. The "surface missed lines loudly" intent that used to live on
the deleted toggle's docs is preserved here, and carried forward as a design input to the
deferred come-back recovery.

The genuinely-wanted recovery — drive back to the node when the seek *overshoots* — is
real but hardware-geometry-coupled and bench-gated, so it does not belong in this
destructive cleanup. It is deferred to a new issue (position-aware come-back recovery),
which will change miss-handling from "hard fault" to "recover, then fault" once built.

## How to apply

- **Resolver:** `WRO2026/StopAtLinePolicy.h::resolveStopAtLine` returns
  `StopAtLineResolution{seek, motionMode}` — no third output. The precedence ladder
  (rungs 1–5) and the 7-field `StopAtLineQuery` are unchanged.
- **Firmware:** the PathTracker blind dispatch builds the auto-seek as one normal
  `Drive.stopAtLine(...)` chain (no `setMaxDistance` override). A no-line fault flows
  through the shared `DRIVE_FAULT` guard with no special branch; there is no
  `_activeSoftFallback` member.
- **Config:** there is no `STOP_AT_LINE_FAULT_FALL_BACK` toggle. `STOP_AT_LINE_MAX_MARGIN_CM`
  (the seek's default margin) and both `LINE_SENSOR_AXLE_OFFSET_*` constants are retained —
  they have live non-soft-fallback users.

## Trigger to revisit

- The position-aware come-back recovery is built (#62) —
  at that point miss-handling changes from "hard fault" to "recover to the node, then
  fault if still lost", and the `Drive` `LINE_NOT_FOUND` fault kind is read to discriminate
  the overshoot case.

## See also

- [ADR-037 — Blind→projection auto stop-at-line](./037-blind-projection-auto-stop-at-line.md) — the auto-rule / per-edge override / field-sourced toggle survive; this ADR supersedes only its soft-fallback sub-decision.
- [ADR-027 — `Drive.stopAtLine()` re-anchors localization; arm-window rule; resolver](./027-drive-stop-at-line.md) — the seek primitive and its hard-fault-on-miss contract this aligns with.
- `WRO2026/StopAtLinePolicy.h` — the `{seek, motionMode}` resolver this ADR restores.
- #62 (**position-aware come-back recovery**) — the wanted overshoot recovery (carries the `LINE_NOT_FOUND` discrimination + fail-loud posture as design inputs); supersedes the recovery-debt tracker, issue #60 (closed).
- #63 (**CSV-driven C++ full-domain parity harness**) — runs the 936-row truth-table CSV through the real `resolveStopAtLine` (until then the C++ tests cover representative rows, not every row).
- `docs/plans/2026-06-16-stop-at-line-softfallback-removal.md` — the originating plan.
