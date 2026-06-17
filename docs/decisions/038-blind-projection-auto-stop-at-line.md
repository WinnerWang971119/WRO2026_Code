# ADR-038: Blind→projection auto stop-at-line — auto-rule, per-edge override, field-sourced toggle, soft-fallback

**Status:** Implemented
**Date decided:** 2026-06 (issue #43, Phase 3)

> **Soft-fallback sub-decision superseded by [ADR-039](./039-stop-at-line-normal-seek-no-soft-fallback.md).** The auto-rule, the per-edge `stop_at_line_override`, and the field-sourced toggle below are still in force; only the soft-fallback (`soft_fallback` output, `_activeSoftFallback` latch, `Config::STOP_AT_LINE_FAULT_FALL_BACK`) is removed — the auto-seek now fires a normal `stopAtLine` and a missed line is a hard fault.

## Rule

A BLIND hop whose resolved next hop (via `NEXT_HOP`) is a `PROJECTION` edge
automatically runs a stop-at-line seek by default, so the chassis re-anchors on a
real tape line before the projection follow instead of dead-reckoning into it.

The resolver (`StopAtLinePolicy.h::resolveStopAtLine`) stays route-agnostic: the
**caller** (PathTracker blind dispatch, Python `_walk_hops`, offline `compile_path`)
computes `auto_rule_active = BLIND_PROJECTION_AUTO_SEEK && next_hop_is_projection`
and passes it in — the resolver never learns the word "projection."

**Precedence ladder** (highest wins):

1. Call-site `StopAtLine ON/OFF`
2. Per-edge `stop_at_line_override ON/OFF` (new `EdgeInfo` field, `StopAtLineMode{DEFAULT,ON,OFF}`)
3. Legacy `BlindMode::STOP_AT_LINE` force-on (kept for backward compat — partial decoupling)
4. Auto-rule (`auto_rule_active`)
5. Otherwise never seek (`requested = false`) — there is intentionally **no** pipeline-default seek rung; the resolved `edgeMode` already encodes the pipeline default, so a call-`AUTO`/edge-`DEFAULT` hop with no rule active falls straight through to `seek = false`.

`stop_at_line_override` is **directional**: baked on the forward `u→v` edge only, never
mirrored to `v→u`, matching the `direction_override` convention. Control the reverse
sense with a swapped EDGES entry.

The `soft_fallback` output flag is true only when the auto-rule fires AND `board_ok`;
explicit/call-ON/legacy seeks always hard-fault regardless of the Config toggle.

## Why

Dead-reckoned BLIND hops accumulate encoder drift (Risk #9). A PROJECTION follow that
begins mid-segment has no line to anchor to — the chassis must start the projection at a
known lateral position. Automatically seeking the preceding junction line closes this gap
without requiring per-edge authoring on every BLIND→PROJECTION pair in the field.

The `auto_rule_active` indirection keeps the resolver truth table bounded: the resolver
never branches on edge type, so adding future edge types does not change the resolver
code or CSV. The field-wide toggle `blind_projection_auto_seek` lives in
`field_data.json` and is emitted as a standalone constant `Routes::BLIND_PROJECTION_AUTO_SEEK`
in `routes.h`, not `Config.h` — it travels with the field geometry, not the firmware, unlike
the firmware-sourced `PATH_PASS_THROUGH_ENABLED` / `BIDIRECTIONAL_AUTO_ENABLED` bools.
It is a standalone constant, not a `PipelineConfig` field, because `PipelineConfig` is a
5-enum motion bundle whose `__post_init__` rejects non-enum members.

The dedicated per-edge `stop_at_line_override` decouples seek from motion so a future GUI
maps one field to motion mode and one field to seek. Decoupling is **partial** by design:
the legacy `BlindMode::STOP_AT_LINE` tag still force-seeks (backward compat); the
`stop_at_line=OFF` rung sits above it in the ladder, so OFF always suppresses a legacy force-on.

The soft-fallback (`Config::STOP_AT_LINE_FAULT_FALL_BACK`, default `false`) is
implemented but gated off during development to surface missed lines loudly. When enabled
at competition, a no-line fault on an auto-triggered seek degrades to dead-reckon: the
recovery issues an explicit corrective `driveDist` for the axle-to-sensor offset before
`onDeadReckonedAdvance` — the overshoot roll runs only on detection, never on a fault, so
the cap alone (`setMaxDistance(distance_cm)`) does not land the axle on the node.

## How to apply

- **Authoring (offline):** set top-level `"blind_projection_auto_seek": true/false` in
  `field_data.json`. Add `"stop_at_line": "on"/"off"` on specific EDGES entries in
  `field_data.py` (directional, forward sense only — GUI wiring deferred to issue #52).
- **Offline pipeline:** `field_data.py` exposes `BLIND_PROJECTION_AUTO_SEEK`; the
  validator accepts the optional bool and per-edge string/enum; the graph builder extracts
  `stop_at_line_override` as a directional attr (never mirrored, like `direction`);
  `header_writer.py` emits `enum class StopAtLineMode`, appends `stop_at_line_override`
  to `EdgeInfo` (both `_edge_cell` and `_none_edge_cell` builders), and emits the
  standalone `constexpr bool BLIND_PROJECTION_AUTO_SEEK`.
- **Firmware (`PathTracker`):** at BLIND dispatch, peek the next-next node via
  `FieldMap::getNextHop` and check its edge type; fold with the toggle into
  `auto_rule_active`; build `StopAtLineQuery` struct and call `resolveStopAtLine`; latch
  `_activeSoftFallback`. In the Drive-fault guard, before entering `DRIVE_FAULT`, check
  all four conditions (`_state`, `_activeSoftFallback`, `Config::STOP_AT_LINE_FAULT_FALL_BACK`,
  fault kind) and execute the soft-recovery sequence: `Drive.clearFault()` → corrective
  advance → `onDeadReckonedAdvance` → `PLAN_NEXT_HOP`.
- **FieldMap:** add `Routes::StopAtLineMode::DEFAULT` as the trailing initializer in the
  `FieldMap::noEdge()` positional `EdgeInfo` init.
- **Cross-language parity:** `StopAtLinePolicy.h::resolveStopAtLine` and
  `sim/runtime.py::resolve_stop_at_line` are step-for-step identical. A checked-in
  generator produces a 936-row truth-table CSV (864 base + 72 supplemental) as a third
  independent transcription; TC13 (Python) and TC13-CPP (C++) both assert agreement
  against every row.

## Trigger to revisit

- Full motion/seek orthogonality is wanted — requires removing or deprecating
  `BlindMode::STOP_AT_LINE` (backward-compat break).
- GUI authoring of per-edge `stop_at_line` and the field-wide toggle is implemented in
  issue #52 — at that point the ADR-022 boundary (EDGES-in-JSON) is reopened.
- Hardware bench confirms the exact soft-recovery corrective advance (the
  `LINE_SENSOR_AXLE_OFFSET_CM` offset is geometry, bench-verified, not asserted here).
- `STOP_AT_LINE_FAULT_FALL_BACK` is flipped to `true` before competition — a deliberate,
  reviewed flip, not a silent change.

## See also

- [ADR-027 — `Drive.stopAtLine()` re-anchors localization; arm-window rule; resolver](./027-drive-stop-at-line.md) — the seek primitive and the existing 3-layer resolver this ADR extends.
- [ADR-022 — `field_data` geometry lives in JSON; `field_data.py` is a loader](./022-field-data-json-loader.md) — per-edge EDGES-in-JSON deferred to issue #52.
- [ADR-024 — Bidirectional line following — 3-layer override precedence](./024-bidirectional-line-following.md) — the `direction_override` directional convention mirrored here.
- [ADR-029 — The validator is the single anti-ambiguity gate](./029-validator-anti-ambiguity.md) — extended with the optional bool + per-edge `stop_at_line` key.
- [ADR-031 — Config split per domain](./031-config-split-per-domain.md) — why `BLIND_PROJECTION_AUTO_SEEK` goes to `routes.h`, not `Config.h`.
- `WRO2026/StopAtLinePolicy.h` — the single C++ source of the resolver (extended by this ADR).
- `docs/plans/2026-06-15-issue43-auto-stop-at-line-projection.md` — the originating plan.
- Issue #43 (closed by this), issue #52 (deferred GUI authoring).
