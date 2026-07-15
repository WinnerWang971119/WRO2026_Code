# ADR-064: Per-edge `primitive_mode` splits into a uniform Speed axis + a type-switched Mode axis; `STOP_AT_LINE` folds into `stop_at_line`

**Status:** Implemented
**Date decided:** 2026-07 (phase)

## Rule

The `EDGES` override key `primitive_mode` (ADR-047) is retired and its two conflated
concerns become independent axes:

- **Speed** — uniform across edge types. One JSON key `speed`, one enum `SpeedTier
  {DEFAULT, SLOW, NORMAL, FAST}`, one `EdgeInfo.speed_override` field (the renamed
  `line_mode_override` slot — `EdgeInfo` stays 7 fields). Resolved for **both** LINE and
  BLIND hops, including the BLIND seek dispatch (`Drive.stopAtLine(...).setPower(...)`),
  which no longer hardcodes `Config::SPEED_NORMAL`.
- **Mode** — type-switched, carried in JSON `mode`. LINE accepts only `MIDDLE`
  (surface-only, see below); BLIND accepts `SYNC_TSHAPE`/`GYRO_TSHAPE`/`RAW_FLAT` into
  the existing `blind_mode_override` field.

`primitive_mode` is hard-removed with no alias — a targeted validator error names `mode`
+ `speed` and this ADR instead of falling through to a generic unknown-key rejection.
`STOP_AT_LINE` is no longer an authorable Mode value, per-edge or as `pipeline_blind_mode`
— a seek is authored solely via `stop_at_line: on`. Legacy `primitive_mode: STOP_AT_LINE`
lowers to the **explicit** canonical form `mode: SYNC_TSHAPE` + `stop_at_line: on` —
never bare `stop_at_line: on`, which would leave `blind_mode_override = DEFAULT` and
reopen the pipeline-inherited fork below.

**The fold is not unconditionally byte-identical.** `StopAtLinePolicy::resolveStopAtLine`
gives a bare `STOP_AT_LINE` edge that seeks (`result.seek`) `motionMode = SYNC_TSHAPE`
(`StopAtLinePolicy.h:173-174` — it has no velocity profile of its own). But the
*degraded*, non-seeking path (REAR seek on a 1-board chassis, or the call-site/override
OFF) fell back to `q.pipelineBlind` (`StopAtLinePolicy.h:182-183`) — whatever
`PIPELINE_CONFIG.blind` was — before its own residual-`STOP_AT_LINE` guard degraded that
to `SYNC_TSHAPE`. The explicit lowering sets `edgeMode = SYNC_TSHAPE` directly, so the
degraded path now also resolves `base = SYNC_TSHAPE` unconditionally, **pinning** it to
match the already-seeking case instead of inheriting the pipeline default. The one case
this changes: a degraded legacy-`STOP_AT_LINE` hop under a **non-SYNC**
`PIPELINE_CONFIG.blind` previously dead-reckoned under that non-SYNC mode and now
dead-reckons under SYNC. This is an accepted, intentional pin — a seek and its own
fallback stay on one motion mode — not an incidental equivalence. Today's default
(`PIPELINE_CONFIG.blind = SYNC_TSHAPE`) makes current behavior unchanged.
`pipeline_blind_mode: STOP_AT_LINE` is also rejected, so the pipeline rung feeding that
fork is fully unauthorable.

LINE Mode is **surface-only this PR**: `mode: MIDDLE` validates and emits nothing (no
`EdgeInfo` field, no C++ enum); `LEFT`/`RIGHT` are validator-rejected pending the
lateral-follow PR (issue #156). Blind-chain eligibility gains a resolved-**speed-tier**
compare only — a differing tier breaks the chain, but a SYNC↔GYRO mode boundary still
chains exactly as it did before this ADR.

## Why

`primitive_mode` overloaded two independent concerns onto one key: a speed the operator
wants and a motion-mode/seek bundle the firmware runs. Issue #97 (per-edge BLIND speed —
today hardcoded to `SPEED_NORMAL`) and issue #99 (the editor exposing two independent
controls) both hinge on the same data-shape decision, so they are absorbed into one
combined feature rather than reworking the field twice. The JSON break is free right
now — only fixtures author `EDGES`, the real field has none — so the migration and the
`STOP_AT_LINE` fold land in the same PR instead of a second breaking change later.

## How to apply

- Author speed via `speed: SLOW|NORMAL|FAST` (both edge types, bidirectional); author
  mode via `mode:` (type-switched, bidirectional); author a seek via `stop_at_line: on`
  only. Never author `primitive_mode` or `mode: STOP_AT_LINE`.
- `PIPELINE_CONFIG.line` renames to `PIPELINE_CONFIG.speed` (JSON `pipeline_speed`,
  default `NORMAL`), applying to both edge types at both the firmware and
  `sim/runtime.py` resolution surfaces.
- Resolve BLIND speed through one mapping, `speedTierToPreset` (next to
  `presetToPwm` in `WROMath.h`) — no second PWM table.
- The blind-chain predicate (`_chainTryStart` / walk-ahead) compares resolved
  `SpeedTier` only; do not add a mode-equality restriction.

## Trigger to revisit

- LineFollower lateral offset-follow lands (issue #156) — `LEFT`/`RIGHT` become
  authorable, adding a `line_follow_mode_override` `EdgeInfo` field; this ADR's
  "surface-only, no field" clause for LINE Mode is superseded at that point.
- Per-edge numeric PWM authoring (`speed_pwm`) is requested — today's tiers keep field
  data chassis-agnostic (TT vs Lofaix); a numeric override would need its own ADR.

## See also

- [ADR-047 — JSON-native EDGES overrides + field-sourced PIPELINE_CONFIG](./047-json-edges-overrides-and-pipeline-config.md) — the override overlay this ADR re-shapes; its `primitive_mode` string-tag ENCODING clause is superseded here, the rest (JSON-native `EDGES`, field-sourced `PIPELINE_CONFIG`, optional geometry) stands.
- [ADR-048 — Reset nodes are blind-incident only; STOP_AT_LINE contradiction](./048-reset-blind-incidence-and-stop-at-line-contradiction.md) — its `STOP_AT_LINE` + `stop_at_line='off'` contradiction rule is dissolved here (inexpressible once `STOP_AT_LINE` cannot be a Mode); the reset-incidence rule stands.
- [ADR-038 — Blind→projection auto stop-at-line](./038-blind-projection-auto-stop-at-line.md) / [ADR-039](./039-stop-at-line-normal-seek-no-soft-fallback.md) — `StopAtLinePolicy::resolveStopAtLine`, whose degrade path this ADR pins to SYNC.
- [ADR-059 — Collinear run continuous motion](./059-collinear-run-continuous-motion.md) — the blind-chain eligibility this ADR restricts on speed tier only.
- `docs/plans/2026-07-14-issue-97-99-per-edge-mode-speed.md` — originating plan (Decisions section).
- Issues #97 (absorbed) and #99 (closed by this).
