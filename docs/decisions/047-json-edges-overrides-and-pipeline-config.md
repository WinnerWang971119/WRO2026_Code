# ADR-047: JSON-native EDGES overrides + field-sourced PIPELINE_CONFIG; ast_rewrite retired

**Status:** Implemented
**Date decided:** 2026-06 (issue #79, Phase 3)

> **`primitive_mode` string-tag ENCODING clause superseded by [ADR-064](./064-per-edge-mode-speed-orthogonalization.md).** JSON-native `EDGES` overrides, field-sourced `PIPELINE_CONFIG`, and optional override geometry all still stand; only the `primitive_mode` key itself — the single field conflating speed and motion-mode/seek — is retired. It splits into a uniform `speed` key + a type-switched `mode` key; `STOP_AT_LINE` is no longer an authorable mode (see ADR-064).

## Rule

`EDGES` override entries live in `field_data.json` — `field_data.py` reads
`EDGES = _data.get("EDGES", [])`. The `primitive_mode` field uses a **string-tag encoding**:
the enum member name verbatim (`LineMode` → `NORMAL`/`SLOW`/`FAST`; `BlindMode` →
`SYNC_TSHAPE`/`GYRO_TSHAPE`/`RAW_FLAT`/`STOP_AT_LINE`). Enum-instance authoring in
Python fixtures remains valid; the validator accepts both forms. Geometry (`distance_cm` /
`heading_deg`) is **optional** in an override entry — derived from node coords at build time.

`PIPELINE_CONFIG` defaults are field-sourced via four optional JSON keys
(`pipeline_line_mode` / `pipeline_blind_mode` / `pipeline_turn_mode` /
`pipeline_direction_mode`); absent keys fall back to `pipeline_config.py` defaults, keeping
committed routes byte-identical. The Tk field editor owns `EDGES` (via the `EDGE OVERRIDES`
inspector list); `ast_rewrite.py` (LibCST EDGES rewriter) is retired.

## Why

ADR-022 deferred JSON EDGES representation because "JSON would require a name-to-enum mapping
layer." That trigger fired: issue #79 requires the GUI to author per-edge overrides, and the
1:1 string-tag encoding (`"FAST"` → `LineMode::FAST`) is already handled by
`header_writer._override_enum_value` and `graph_builder`'s verbatim-forward pass — only the
validator needed to accept the string form. The simplicity argument that originally kept EDGES
in Python dissolves once the LibCST rename cascade (`ast_rewrite.py`) is the only remaining
reason to maintain a Python-side literal.

Sourcing `PIPELINE_CONFIG` from the hashed JSON (rather than the unhashed `pipeline_config.py`
singleton) corrects the SHA256 coverage gap: the header now faithfully reflects every input
that affects `routes.h` / `routes.cpp`.

## How to apply

- **Override entries:** author in `field_data.json` as `{"type":"LINE","u":"A","v":"B","primitive_mode":"FAST"}`.
  Omit `distance_cm` / `heading_deg` — derived from node coords. Identity is `(type, u, v)`;
  a directed pair is not unique across types. `direction` ∈ `{"front","rear"}` and
  `stop_at_line` ∈ `{"on","off"}` are forward-only (`u→v`).
- **Pipeline defaults:** add `"pipeline_line_mode"` / `"pipeline_blind_mode"` /
  `"pipeline_turn_mode"` / `"pipeline_direction_mode"` (string tag) as optional top-level
  JSON keys. Absent → `pipeline_config.py` default; line/blind/turn may not be `DEFAULT`.
- **GUI:** use the `EDGE OVERRIDES` collapsible list in the inspector. Edge type is
  auto-derived from the selected segment (read-only); `primitive_mode` combobox values
  switch on the derived type. `PIPELINE DEFAULTS` section controls the four pipeline keys.
- **Validator:** rejects unknown tags, wrong-type tags (LineMode on a BLIND edge), and the
  literal `"DEFAULT"` name for `primitive_mode` or any pipeline key where DEFAULT is banned.
- **`ast_rewrite.py` is deleted.** Node renames cascade into `EDGES` through `FieldModel`
  (model-level `u`/`v` update in `RenameNode`), not via LibCST source rewriting.

## Trigger to revisit

- ~~A `primitive_mode` enum gains a member whose name collides with a JSON keyword or cannot
  be expressed verbatim~~ — moot: `primitive_mode` is retired by ADR-064.
- ~~Per-edge blind speed / numeric PWM is added (issue #97)~~ — fired: ADR-064 lands per-edge
  blind speed (`speed`/`SpeedTier`); numeric PWM authoring stays deferred.

## See also

- [ADR-022 — field_data geometry lives in JSON; field_data.py is a loader](./022-field-data-json-loader.md) — partially superseded (JSON-can't-hold-enums and PIPELINE_CONFIG-out-of-GUI-scope clauses replaced here).
- [ADR-029 — The validator is the single anti-ambiguity and input-finiteness gate](./029-validator-anti-ambiguity.md) — string-tag rejection lives here.
- [ADR-046 — Projection is a node property](./046-projection-as-node-property.md) — the `blind_projection_auto_seek` field-wide toggle is surfaced in the same GUI refactor.
- [ADR-064 — Per-edge mode/speed orthogonalization](./064-per-edge-mode-speed-orthogonalization.md) — supersedes the `primitive_mode` string-tag ENCODING clause above (splits it into `speed` + `mode`); the rest of this ADR stands.
- `docs/plans/2026-06-23-issue-79-edges-json-overrides-gui.md` — originating plan (decisions section).
- Issue #79 (closed by this).
- Issue #97 — per-edge blind speed (absorbed into ADR-064).
