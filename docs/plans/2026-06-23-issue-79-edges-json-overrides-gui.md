# Issue #79 — JSON-representable per-edge EDGES overrides + operator-tunable inspector

**Goal:** Give the per-edge `EDGES` overrides a JSON-representable encoding so the Tk field
editor can author them, and surface the field/planner tunables (per-edge overrides,
`PIPELINE_CONFIG` defaults, field-wide flags, field dimensions) in the inspector — so an
operator can tune the field graph entirely in the GUI, without breaking enum-typed Python
authoring and without firmware changes.

**Approach:** "JSON-native EDGES + field-sourced pipeline config, resolved at the
validator/graph_builder seam, surfaced via a base-C inspector panel." JSON becomes the EDGES
source of truth (`field_data.py` reads `EDGES` from the JSON instead of hardcoding `[]`).
`primitive_mode` gains a string-tag encoding (enum member names verbatim) accepted by the
validator — `graph_builder` already forwards it and `header_writer._override_enum_value`
already resolves a string name, so only the validator must change. `PIPELINE_CONFIG` defaults
become four optional JSON keys the emitter reads (defaults when absent → committed routes stay
byte-identical). `FieldModel` owns `edges` + the pipeline keys, mirroring the existing
`reset_nodes` precedent through both serialization surfaces. The inspector gets an
`EDGE OVERRIDES` list (base-C, mirroring the Pivots list) plus a `PIPELINE DEFAULTS` section and
field-wide toggles. The now-dead `ast_rewrite.py` LibCST rewriter is retired.

This approach was independently endorsed by two cross-vendor consults (Fable→Opus and Codex/GPT)
with no veto.

## Scope

- **In scope:**
  - **Data layer:** `primitive_mode` JSON string-tag encoding (enum names verbatim); validator
    accepts an enum instance OR a valid tag; `distance_cm`/`heading_deg` become **optional** in an
    EDGES entry (override-only overlays; geometry derived from node coords downstream).
  - **JSON as EDGES source of truth:** `field_data.py` reads `EDGES = _data.get("EDGES", [])`;
    loader key-order + optional-keys carry `EDGES` (absent-stays-absent round-trip).
  - **PIPELINE_CONFIG field-authorable:** four optional JSON keys (`pipeline_line_mode` /
    `pipeline_blind_mode` / `pipeline_turn_mode` / `pipeline_direction_mode`, string tags);
    `header_writer` emits from field-sourced values with `pipeline_config.py` fallback; the stale
    "PIPELINE_CONFIG not in SHA256" comment is corrected (the JSON source is hashed).
  - **Editor model/loader plumbing:** `FieldModel` carries `edges` + 4 pipeline keys through
    `from_json_dict` / `to_json_dict` / `to_module_proxy` / `_JSON_KEY_ORDER` / `snapshot`+`restore`.
  - **GUI (inspector):** base-C `EDGE OVERRIDES` collapsible list (add/edit/delete; `primitive_mode`
    / `direction` / `stop_at_line` controls; `type` auto-derived from the selected segment, locked;
    geometry not shown); graft the editor's existing Edge mode to locate/pre-fill an edge's row on
    click; new `PIPELINE DEFAULTS` section; field-wide `blind_projection_auto_seek` toggle; `FIELD_MM`
    dimensions control. New editor `Command`s (apply/revert), undo/redo integrated.
  - **Retire `ast_rewrite.py`** (+ its tests + the `migrate.py` / `app.py` call sites): once EDGES is
    JSON, node renames flow through `FieldModel.nodes` and the LibCST EDGES rewriter is dead code.
  - **Validator + tests + goldens**, keeping enum-instance Python authoring valid.
  - **ADR-047** recording the JSON encoding decision; supersedes ADR-022's "JSON can't hold the
    enums" and "PIPELINE_CONFIG out of GUI scope" clauses.

- **Out of scope:**
  - **Per-edge blind speed / numeric PWM** — firmware-coupled; filed as **issue #97**.
  - **Firmware / C++ / routes.h/.cpp content changes** — none; committed routes stay byte-identical
    (production field omits the new optional keys → defaults emit unchanged).
  - **MAP_PNG editing** — user wants field dimensions only.
  - **Geometry (distance/heading) hand-editing in the GUI** — derived from node coords.
  - **New-edge creation from the panel** — overrides attach to edges that already exist in
    `LINE_SEGMENTS` / `BLIND_EDGES` only.
  - **customtkinter restyle** — separate issue #83.
  - **`pre_turn_mode` / per-edge `pivot` keys** — retired (#44); validator already rejects them.
  - **PROJECTION as an edge type** — retired (#64); projection is a node property.

## Decisions

- **JSON is the EDGES source of truth** (user pick) — `field_data.py` reads JSON `EDGES`; the Python
  enum-instance form still validates (tests/fixtures) but production has no hand-authored Python
  EDGES literal. Matches ADR-022's stated revisit trigger. Rejected: a Python literal + JSON merge
  (two sources / dedup-conflict risk).
- **`primitive_mode` tags = enum member names verbatim** (user pick) — `"NORMAL"`/`"SLOW"`/`"FAST"`
  (LineMode), `"SYNC_TSHAPE"`/`"GYRO_TSHAPE"`/`"RAW_FLAT"`/`"STOP_AT_LINE"` (BlindMode). 1:1 with the
  C++/Python enum names; `header_writer._override_enum_value` already resolves a name.
- **Resolution lives at the validator/graph_builder seam, not the loader** — mirrors how
  `direction` / `stop_at_line` already resolve; keeps the editor/`FieldModel` enum-free (they only
  ever hold string tags). `graph_builder` already forwards `primitive_mode` verbatim and the emitter
  resolves it — so only the validator must accept the string form.
- **Geometry is optional in an EDGES override entry** (derived from node coords) — the editor writes
  override-only entries `{type, u, v, primitive_mode?, direction?, stop_at_line?}`; `graph_builder`
  already computes `distance_cm`/`heading_deg` from coords when absent. This avoids stale pinned
  geometry when nodes move and minimizes the dual-serialization surface. Hand-authored entries that
  DO carry geometry still validate (backward compatible).
- **PIPELINE_CONFIG field-sourced via the JSON (already hashed)** (Fable amendment) — the four keys
  live in `field_data.json`, which `compute_source_hash` already hashes (via `extra_source_paths`);
  the emitter reads field-sourced values with `pipeline_config.py` fallback. Absent keys → defaults
  emit, so committed routes are byte-identical. NOTE: the header_writer "PIPELINE_CONFIG … NOT part of
  the SHA256 input" comment is **accurate today** (the singleton comes from `pipeline_config.py`, which
  is not hashed) and only becomes wrong once T3 moves the source into the hashed JSON — T3 corrects it
  as part of the same change, not before.
- **Retire `ast_rewrite.py`** (Fable amendment) — its only surviving job (rename `u`/`v` in the
  Python EDGES literal) is dead once EDGES is JSON.
- **GUI edge `type` is auto-derived from the selected segment, read-only** (Fable caution) — prevents
  an operator authoring a conflicting-type override that the validator rejects at Save.
- **UI: base-C override list + edge-mode click graft** (user pick) — reuses the Pivots-list idiom and
  the existing Edge mode; no new canvas selection subsystem. See the `## UI/UX Specification` section.
- **No firmware changes** — `EdgeInfo` already has the four override slots; `header_writer` already
  emits string-resolved overrides; per-edge speed is deferred to #97.

## Acceptance Criteria

- [ ] AC1: A `field_data.json` with an EDGES entry `{"type":"LINE","u":"A","v":"B","primitive_mode":"FAST"}`
      loads, validates, and `build_graph` resolves the edge's `primitive_mode` so `header_writer`
      emits `LineMode::FAST` for that edge. (string-tag primitive_mode)
- [ ] AC2: The same entry with `distance_cm`/`heading_deg` **omitted** validates, and `build_graph`
      derives geometry from node coordinates. (optional geometry)
- [ ] AC3: An EDGES entry authored as a Python enum instance (`primitive_mode=LineMode.FAST`) still
      validates and emits identically. (legacy Python authoring preserved)
- [ ] AC4: A `field_data.json` carrying `"pipeline_blind_mode":"GYRO_TSHAPE"` causes `routes.cpp`'s
      `PIPELINE_CONFIG` to emit `BlindMode::GYRO_TSHAPE`; with the key absent it emits the
      `pipeline_config.py` default (`SYNC_TSHAPE`) and the toy golden is byte-identical.
- [ ] AC5: `validate()` rejects: an unknown `primitive_mode` tag; a `LineMode` tag on a BLIND edge
      (and vice versa); a `DEFAULT`-named tag; `pipeline_line_mode:"DEFAULT"` (line/blind/turn may not
      be DEFAULT); each with a precise `FieldDataError` message.
- [ ] AC6: Editor open→save round-trip is byte-identical (`dump(load(dump(x))) == dump(x)`)
      **unconditionally** — including a field whose EDGES entry carries `distance_cm`/`heading_deg`
      (the model stores entries verbatim, so geometry-carrying hand-authored entries are preserved,
      not normalized); a field WITHOUT the new keys round-trips with them still absent
      (absent-stays-absent).
- [ ] AC7: `FieldModel.to_module_proxy()` exposes `EDGES` + `PIPELINE_*` so `validate()` runs against
      the edited values (dual-serialization parity test passes).
- [ ] AC8: In the editor, adding an `EDGE OVERRIDES` row for an existing line edge, choosing
      `primitive_mode=FAST`, and saving writes the override to `field_data.json`; undo removes it;
      redo re-adds it.
- [ ] AC9: Clicking an existing edge in Edge mode locates/highlights its override row if present, else
      pre-fills the add-row's `u`/`v`/`type` from the clicked segment.
- [ ] AC10: The `PIPELINE DEFAULTS` comboboxes, the `blind_projection_auto_seek` toggle, and the
      `FIELD_MM` control read from and write to the model, persist to JSON, and undo/redo.
- [ ] AC11: `ast_rewrite.py` and `tests/test_editor_ast_rewrite.py` are deleted; no module imports
      `ast_rewrite`; the editor Save no longer rewrites `field_data.py`; a node rename in a field with
      EDGES correctly updates the EDGES `u`/`v` in the model/JSON.
- [ ] AC12: Full `uv run --extra dev pytest tools/path_planner/tests/`, the host `ctest`, the
      `arduino-cli` compile, and the routes-freshness check all pass; `routes.h`/`routes.cpp` are
      unchanged (or regenerated+committed if a deliberate exemplar is added — see Notes).
- [ ] AC13: ADR-047 exists, `DECISIONS.md` indexes it (+ status table), and ADR-022
      (`022-field-data-json-loader.md`) is marked partially superseded.
- [ ] AC14: `edit_field.py --migrate` on a `field_data.py` carrying a legacy Python `EDGES = [...]`
      literal moves those entries into `field_data.json`'s `EDGES` key (override-only string-tag form)
      and emits `EDGES = _data.get("EDGES", [])` — no Python EDGES tail is re-appended.

## UI/UX Specification

> Carried verbatim from the Phase 1.5 design gate. The executor reproduces THIS — do not re-design.
> Full raw output (drawing, all states, hex tokens, type/spacing) lives in the sidecar
> `docs/plans/2026-06-23-issue-79-edges-json-overrides-gui.ui.md`.

**Chosen base:** Direction **C — Override-list manager** (a collapsible `EDGE OVERRIDES` list in the
inspector that mirrors the existing **Pivots** list; add/edit/delete rows; decoupled from canvas
selection; no new selection subsystem).

**Grafted in:** Direction **A's click-to-locate, via the EXISTING Edge mode** — clicking an existing
edge in the editor's current Edge mode locates/highlights that edge's override row (and pre-fills the
add-row's `u`/`v`/`type`). Reuses `_edge_selection`; no new canvas selection state. (User: "C as base
but can click A. Just use the edge mode we already have.")

### Layout (drawing)

```
+-- PROPERTIES --------------------------------+   bg/surface  #EADFC8
| ...existing Nodes list + Node inspector...   |
| ...existing Reset-spec / Pivots collapsibles |
| -------------------------------------------- |
| v EDGE OVERRIDES                      (2)     |   header #2B2622, chevron #C56B45
|   | B->G  [BLIND]  seek=on          e  x |    |   BLIND tag #B0413E
|   | A->C  [LINE]   FAST              e  x |    |   LINE tag #2B2622
|   + add override                              |
|   +-- inline editor (add / row "e") ------+   |
|   |  Edge  u[B v] v[G v]  Type BLIND(ro)  |   |   type DERIVED from segment, read-only
|   |  Primitive  [ STOP_AT_LINE        v]  |   |   BlindMode tags (blind) / LineMode tags (line)
|   |  Direction  [ default             v]  |   |   default | front | rear
|   |  Stop@line  [ default             v]  |   |   default | on | off
|   |  (distance / heading auto-derived)    |   |   geometry NOT shown
|   |  [ save ]  [ cancel ]                 |   |
| -------------------------------------------- |
| v PIPELINE DEFAULTS                           |
|   Line [NORMAL v] Blind [SYNC_TSHAPE v]       |   line/blind/turn: no DEFAULT
|   Turn [COUNTER_ROT v] Direction [DEFAULT v]  |   direction: DEFAULT allowed
| -------------------------------------------- |
| v GLOBAL KNOBS (existing, extended)           |
|   [x] Blind-decay ...   robot_radius_cm[9.0]  |   existing
|   [x] blind_projection_auto_seek              |   NEW toggle
|   Field size  W[2362] H[1143] mm              |   NEW FIELD_MM (no MAP_PNG)
| -------------------------------------------- |
| ...existing Start pose + action buttons...    |
+----------------------------------------------+
```

### Description

The `EDGE OVERRIDES` collapsible sits below Pivots and mirrors its idiom (count badge, scrollable
rows with per-row edit/delete, a ghost "+ add override", an inline editor sub-frame). The inline
editor sets the edge by `u`/`v` comboboxes; **`type` is auto-derived from the matching existing
segment and shown read-only** so an operator cannot author a conflicting-type override. Controls:
`primitive_mode` (combobox whose values switch on the derived type — LineMode tags for line, BlindMode
tags for blind; `DEFAULT` shown as "default (inherit)" = key omitted), `direction`
(`default`/`front`/`rear`), `stop_at_line` (`default`/`on`/`off`). Distance/heading are never shown —
derived from node coords at build time. `PIPELINE DEFAULTS` is a new collapsible with four mode
comboboxes (line/blind/turn may not be DEFAULT; direction may). `GLOBAL KNOBS` gains a
`blind_projection_auto_seek` checkbutton and a `FIELD_MM` width/height pair.

### Color codes

| Token | Hex | Use |
|-------|-----|-----|
| PAPER | `#F3EAD8` | page / plotting background |
| PAPER_LIGHT | `#FAF4E6` | entry-field background |
| SURFACE | `#EADFC8` | inspector panel surface |
| INK | `#2B2622` | header / body text, LINE tag |
| INK_MUTED | `#6E6457` | secondary text, ghost button rest |
| TERRACOTTA | `#C56B45` | accent: chevrons, hover, selection ring |
| BRICK | `#B0413E` | BLIND edge tint, invalid badge |
| GRID | `#D7C9AC` | section dividers |
| VALID | `#5C8A3A` | validation-OK badge |

(Edge-relevant tokens only; the full palette incl. `OCHRE #C8951F` / `TEAL #2A8C82` node tints is in
the sidecar.)

### Type & spacing tokens

Fonts: `FONT_SANS` (labels/headers), `FONT_MONO` (numeric entries). Title 16 pt, section header 9 pt
small-caps, UI 10 pt (`theme.py`). Rows: `side="top", fill="x"`, left label + right control, 6 px
inter-control padding — matching reset-spec / pivots.

### States

Default / Add-edit / Hover / Loading / Empty / Error as detailed in the sidecar. Save-time validation
failure turns the existing status-strip badge INVALID `#B0413E` with the validator message; obvious
edit-time errors are inline-rejected before dispatch (existing pattern).

### Responsive

Fixed 312 px band; each new section is individually collapsible via the existing
`_toggle_collapsible` mechanism.

## Data Model

### JSON schema additions (`field_data.json`)

```jsonc
{
  // ... existing keys ...
  // NEW optional top-level keys (absent-stays-absent):
  "pipeline_line_mode":      "NORMAL",        // LineMode name; omit/absent = inherit pipeline_config.py default
  "pipeline_blind_mode":     "SYNC_TSHAPE",   // BlindMode name
  "pipeline_turn_mode":      "COUNTER_ROT",   // TurnMode name
  "pipeline_direction_mode": "DEFAULT",       // DirectionMode name (DEFAULT allowed here)
  "EDGES": [
    // override-only overlay entries on existing LINE_SEGMENTS / BLIND_EDGES pairs.
    // distance_cm / heading_deg OPTIONAL (derived from node coords when absent).
    { "type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST" },
    { "type": "BLIND", "u": "B", "v": "G", "stop_at_line": "on" },
    { "type": "BLIND", "u": "S", "v": "A", "direction": "front" }
  ]
}
```

### `primitive_mode` string-tag vocabulary

| Edge type | Valid tags (enum names verbatim) | DEFAULT |
|-----------|----------------------------------|---------|
| `LINE` | `NORMAL`, `SLOW`, `FAST` | absence (do not author `DEFAULT`) |
| `BLIND` | `SYNC_TSHAPE`, `GYRO_TSHAPE`, `RAW_FLAT`, `STOP_AT_LINE` | absence |

`direction` ∈ {`front`, `rear`} (forward `u→v` only). `stop_at_line` ∈ {`on`, `off`} (forward only).

### `FieldModel` new fields (`editor/model.py`)

```python
edges: list[dict] = field(default_factory=list)   # override-only entries (type,u,v,+optional override keys)
pipeline_line_mode: str | None = None
pipeline_blind_mode: str | None = None
pipeline_turn_mode: str | None = None
pipeline_direction_mode: str | None = None
```

## Contracts & Interfaces

Single source of truth for every cross-task seam. A task that touches a seam conforms to the exact
name/shape here; it does not redefine it.

### JSON top-level keys (owner: T2 loader/field_data; consumer: T5 model)

- `"EDGES"` — list of override-only dicts (shape below). Optional; absent-stays-absent.
- `"pipeline_line_mode"`, `"pipeline_blind_mode"`, `"pipeline_turn_mode"`, `"pipeline_direction_mode"`
  — string (enum name) or absent. Optional; absent-stays-absent.
- **Exact final `_TOP_LEVEL_KEY_ORDER` / `_JSON_KEY_ORDER` tuple** (T2 `loader.py` and T5 `model.py` MUST
  use this identical ordering — pinned once here):

  ```
  "FIELD_MM", "MAP_PNG", "ROBOT_RADIUS_CM", "BLIND_DECAY_ENABLED", "BLIND_DECAY_K",
  "BLIND_DECAY_EXPONENT", "BLIND_DECAY_D_REF", "START_NODE", "START_HEADING_DEG",
  "NODES", "NODE_TYPES", "LINE_SEGMENTS", "BLIND_EDGES", "pivot_turns", "RESET_NODES",
  "blind_projection_auto_seek",
  "pipeline_line_mode", "pipeline_blind_mode", "pipeline_turn_mode", "pipeline_direction_mode",
  "EDGES", "OBSTACLES"
  ```
- New keys added to `_OPTIONAL_TOP_LEVEL_KEYS`: the 4 `pipeline_*` keys + `"EDGES"` (absent-stays-absent).

### EDGES entry shape (owners: T1 validator, T2 field_data, T5 model, T6 commands, T7 panels)

```
{ "type": "LINE" | "BLIND",   # required
  "u": <node name>,            # required, exists in NODES
  "v": <node name>,            # required, exists in NODES, != u
  "primitive_mode"?: <tag>,    # LineMode tag on LINE, BlindMode tag on BLIND, or an enum instance (legacy)
  "direction"?: "front"|"rear",
  "stop_at_line"?: "on"|"off",
  "distance_cm"?: <float>,     # OPTIONAL — derived from coords when absent
  "heading_deg"?: <float> }    # OPTIONAL — derived from coords when absent
```

Identity is `(type, u, v)` (a directed pair is NOT unique across types). Overlay entries must use the
**matching** type of the existing segment (EDGE_LINE on a LINE_SEGMENTS pair, EDGE_BLIND on a
BLIND_EDGES pair) — the validator's anti-ambiguity rule rejects a conflicting type.

### `field_data` module attributes (owner: T2; consumers: T1 validator, T3 header_writer)

- `EDGES = _data.get("EDGES", [])` (SCREAMING_SNAKE attr name `EDGES`).
- `PIPELINE_LINE_MODE`, `PIPELINE_BLIND_MODE`, `PIPELINE_TURN_MODE`, `PIPELINE_DIRECTION_MODE`
  = `_data.get("pipeline_*", None)`.

### `header_writer` pipeline read path (owner: T3; producer: T2)

- `_emit_pipeline_config(field_data)` reads `getattr(field_data, "PIPELINE_LINE_MODE", None)` (etc.),
  resolves each via `<EnumCls>[name]`, falls back to the `pipeline_config.py` `PIPELINE_CONFIG`
  default when `None`. The "PIPELINE_CONFIG values … intentionally NOT part of the SHA256 input"
  comment is replaced (field-sourced keys live in the hashed JSON).

### `FieldModel` ↔ validator seam (owner: T5)

- `to_module_proxy()` MUST expose `EDGES=self.edges` AND `PIPELINE_LINE_MODE=self.pipeline_line_mode`
  (etc., SCREAMING_SNAKE) — else the field is persisted-but-unvalidated (the dual-serialization trap).
- `to_json_dict()` MUST emit the same keys (snake/JSON form) under the same absent-when-empty rule.
- Both surfaces emit EDGES as override-only entries (no geometry).

### New editor `Command`s (owner: T6; pattern: `SetResetSpec` analog, apply/revert)

- `SetEdgeOverride(edge_type, u, v, primitive_mode=None, direction=None, stop_at_line=None)`
- `ClearEdgeOverride(edge_type, u, v)`
- `SetPipelineMode(which: "line"|"blind"|"turn"|"direction", value: str|None)`
- `SetFieldWideFlag(flag: "blind_projection_auto_seek", value: bool)`
- `SetFieldDimensions(width_mm: float, height_mm: float)`

### File ownership

| File | Owner | Consumers |
|------|-------|-----------|
| `tools/path_planner/validator.py` | T1 | — |
| `tools/path_planner/field_data.py` (incl. docstring) | T2 | — |
| `tools/path_planner/editor/loader.py` | T2 | — |
| `tools/path_planner/header_writer.py` | T3 | — |
| `tools/path_planner/pipeline_config.py` | T3 | — |
| `tools/path_planner/editor/model.py` | T5 | — |
| `tools/path_planner/editor/commands.py` | T6 | — |
| `tools/path_planner/editor/panels.py` | T7 | — |
| `tools/path_planner/editor/app.py` | T8 | — |
| `tools/path_planner/editor/ast_rewrite.py` (delete), `editor/migrate.py` | T9 | — |
| `tools/path_planner/tests/*` (data) | T10 | — |
| `tools/path_planner/tests/test_editor_*` (GUI) | T11 | — |
| `docs/decisions/047-*.md`, `DECISIONS.md`, `022-field-data-json-loader.md`, `README.md` | T12 | — |

`graph_builder.py` needs **no change** (already forwards `primitive_mode` verbatim and derives
geometry when absent in `_emit_directed_pair`) — verification only, covered by T10. **But the
geometry-optional change is NOT confined to graph_builder:** `validator.py::_build_authored_graph`
(T1's file) has its OWN geometry-absent `continue` skip (lines ~325-326) that must also be patched to
derive instead of skip (see T1). Do not conflate "graph_builder needs no change" with "the whole
geometry path needs no change."

## Error Handling

- **Unknown / wrong-type / DEFAULT tag** (primitive_mode or pipeline mode): `validate()` raises
  `FieldDataError` with a precise key path + fix hint; in the GUI, Save surfaces it on the status
  badge (INVALID `#B0413E`) and tints the offending row.
- **Override on a non-existent / conflicting-type segment:** the GUI derives `type` from the existing
  segment (read-only), so this can't be authored; if it somehow occurs (hand-edited JSON), the
  validator's anti-ambiguity cross-check rejects it at Save.
- **Stale geometry:** impossible by construction — geometry is derived from node coords at build time,
  not stored.
- **Header emit with a bad field-sourced pipeline value:** `<EnumCls>[name]` raises `KeyError`; the
  validator catches it first (`_validate_pipeline_config`), so a build never reaches emit with a bad
  value. If it does (unvalidated fixture), the emitter raises rather than emitting garbage.

## Testing Strategy

**Levels:** Unit (validator, loader, model, header_writer, commands) + Integration (build_routes
end-to-end, editor save→load round-trip, panels smoke).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | EDGES `primitive_mode:"FAST"` on a LINE edge | Integration | validates; `build_graph` resolves; emit `LineMode::FAST` |
| TC2 | EDGES entry with geometry omitted | Unit | validates; `build_graph` derives distance/heading from coords |
| TC3 | EDGES `primitive_mode=LineMode.FAST` (enum instance) | Unit | validates; emits identically (legacy path) |
| TC4 | `primitive_mode:"FAST"` on a BLIND edge | Unit | `FieldDataError` (LineMode tag on BLIND) |
| TC5 | `primitive_mode:"BOGUS"` | Unit | `FieldDataError` (unknown tag) |
| TC6 | `primitive_mode:"DEFAULT"` | Unit | `FieldDataError` (DEFAULT == absence) |
| TC7 | `pipeline_blind_mode:"GYRO_TSHAPE"` | Integration | emit `BlindMode::GYRO_TSHAPE` in `PIPELINE_CONFIG` |
| TC8 | pipeline keys absent → re-run the golden emitter on the unmodified toy SimpleNamespace | Integration | emit `pipeline_config.py` defaults; `routes_toy.{h,cpp}` regenerate with **zero diff** (assert the emitter output equals the committed golden) |
| TC9 | `pipeline_line_mode:"DEFAULT"` | Unit | `FieldDataError` (line/blind/turn may not be DEFAULT) |
| TC10 | loader dump→load→dump with EDGES + pipeline keys | Unit | byte-identical |
| TC11 | loader round-trip of a field WITHOUT the new keys | Unit | keys stay absent (absent-stays-absent) |
| TC12 | `to_module_proxy()` AND `to_json_dict()` BOTH carry EDGES + PIPELINE_* | Unit | `validate()` runs against edited values; assert both surfaces emit the same key set (dual-serialization parity — the #1 risk) |
| TC13 | `SetEdgeOverride` apply/revert | Unit | apply adds entry; revert restores prior state exactly |
| TC14 | `ClearEdgeOverride` / `SetPipelineMode` / `SetFieldWideFlag` / `SetFieldDimensions` apply/revert | Unit | round-trip invariant holds |
| TC15 | panels smoke after restyle | Integration (real Tk, xvfb) | new sections + callback slots present |
| TC16 | edge-mode click locates/pre-fills override row | Integration | row highlighted or add-row pre-filled from segment |
| TC17 | node rename in a field with EDGES | Unit | EDGES `u`/`v` updated in model/JSON (no `ast_rewrite`) |
| TC18 | no module imports `ast_rewrite`; file + its test deleted | Unit/CI | import scan clean; ctest+pytest green |
| TC19 | `build_routes` freshness | Integration | regenerate → `git diff --exit-code` clean |
| TC20 | an override-only EDGES entry (no geometry) that is ALSO a pivot leg | Unit | `_validate_pivot_turns` still sees the edge (geometry derived, not skipped); pivot classification + `stop_at_line`/`direction` on that leg resolve correctly (guards the `_build_authored_graph` fix) |
| TC21 | a node rename in a field that has an EDGES override on the renamed node | Unit | `RenameNode` cascades into `model.edges` so the entry's `u`/`v` follow the rename (replaces the deleted `ast_rewrite` path) |
| TC22 | `--migrate` on a Python `EDGES = [...]` literal | Integration | entries land in `field_data.json` `EDGES`; regenerated `field_data.py` has no Python EDGES tail (AC14) |

**Test data:** extend `tests/fixtures/field_data_stop_at_line.json` patterns; a small in-test field
with one LINE + one BLIND override; reuse `_clone_field_data` SimpleNamespace fixtures (they keep
working because EDGES + pipeline reads use `getattr(..., default)`); editor model fixtures mirror the
existing `reset_nodes` round-trip tests.

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/` (Python);
`ctest --test-dir build/test` (host C++); `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Validator: string-tag `primitive_mode` + optional geometry (incl. `_build_authored_graph`) + pipeline-key validation | — | high | `tools/path_planner/validator.py` | In `_validate_edges_field`: accept `primitive_mode` as an enum instance OR a valid `LineMode`/`BlindMode` member NAME (cross-checked by edge type). **Reject the literal `"DEFAULT"` name explicitly** — `LineMode["DEFAULT"]` SUCCEEDS, so the existing `== expected_cls.DEFAULT` check (which only catches enum instances) must be extended to also reject the string `"DEFAULT"` before/after `enum_cls[name]` resolution; reject unknown names with precise `FieldDataError`. Make `distance_cm`/`heading_deg` OPTIONAL in `_validate_edges_field` (skip the required-key check; keep range checks when present). **ALSO patch `_build_authored_graph` (lines ~325-326): it currently `continue`s past any entry lacking `distance_cm`/`heading_deg`, which would silently drop every override-only entry (incl. pivot legs) from the pivot-validation graph — change it to derive geometry from node coords (via `_emit_directed_pair`) instead of skipping.** Add `_validate_pipeline_config(field_data)` reading optional `PIPELINE_LINE_MODE`/`_BLIND_MODE`/`_TURN_MODE`/`_DIRECTION_MODE` (string name → enum; line/blind/turn reject DEFAULT, direction allows it; mirror `PipelineConfig.__post_init__`); call it from `validate()`. Satisfies AC1–3,5; TC1–6,9,20. |
| T2 | JSON as EDGES source of truth + loader/field_data keys + migrate.py EDGES-tail retirement | — | med | `tools/path_planner/field_data.py`, `tools/path_planner/editor/loader.py`, `tools/path_planner/editor/migrate.py` | `field_data.py`: replace `EDGES = []` with `EDGES = _data.get("EDGES", [])`; add `PIPELINE_LINE_MODE`/`_BLIND_MODE`/`_TURN_MODE`/`_DIRECTION_MODE = _data.get("pipeline_*", None)`; rewrite the EDGES authoring docstring (JSON, string tags, optional geometry). `loader.py`: add `"EDGES"` + the 4 `pipeline_*` keys to `_TOP_LEVEL_KEY_ORDER` and `_OPTIONAL_TOP_LEVEL_KEYS` — use the **exact final tuple pinned in Contracts §"JSON top-level keys"**; confirm `_convert_arrays_to_tuples` passes EDGES dicts through unchanged. `migrate.py`: the EDGES-tail machinery (`_EDGES_BANNER`, `_extract_edges_tail`, `_EDGES_ASSIGN_TOKEN`, and the `_build_rewritten_source` template that preserves the Python `EDGES = [` block verbatim) is now obsolete — the legacy Python EDGES literal must migrate INTO the JSON `EDGES` key, not be preserved as a Python tail. Update `migrate.py` so a one-shot `--migrate` carries any existing Python EDGES entries into `field_data.json` (as override-only string-tag dicts) and emits `EDGES = _data.get("EDGES", [])` rather than re-appending the literal tail. Satisfies AC1,6,14; TC10–11. |
| T3 | Header_writer: field-sourced `PIPELINE_CONFIG` + SHA256 comment fix | T2 | high | `tools/path_planner/header_writer.py`, `tools/path_planner/pipeline_config.py` | `_emit_pipeline_config(field_data)` reads `getattr(field_data, "PIPELINE_*", None)`, resolves via `<EnumCls>[name]`, falls back to the `pipeline_config.py` `PIPELINE_CONFIG` default when `None`; thread `field_data` into its caller (`_build_routes_cpp_lines`). Replace the stale "PIPELINE_CONFIG … NOT part of the SHA256 input" comment (now field-sourced from the hashed JSON). Optional `pipeline_config.PipelineConfig.from_field_data(field_data)` helper reusing `__post_init__`. Satisfies AC4; TC7–8. |
| T5 | `FieldModel` carries `edges` + 4 pipeline keys (both serialization surfaces) | — | high | `tools/path_planner/editor/model.py` | Add `edges: list[dict]` + 4 `pipeline_*: str|None` fields (defaulted). Thread through `_JSON_KEY_ORDER`, `from_json_dict` (read `EDGES` entries **verbatim** — store each dict as-is, preserving whatever keys are present incl. any geometry, so geometry-carrying hand-authored entries round-trip byte-identically; read pipeline keys), `to_json_dict` (emit EDGES entries verbatim + pipeline keys; absent-when-empty), `to_module_proxy` (expose `EDGES` + `PIPELINE_*` SCREAMING_SNAKE — the dual-serialization seam), `snapshot`/`restore` (deep-copy `edges`). Mirror the `reset_nodes` precedent (verbatim-preserve; NOTE the model stores EDGES verbatim — it does NOT normalize/drop geometry, so byte-identical round-trip holds unconditionally; the editor's *new* overrides are authored override-only by T6, not normalized here). Satisfies AC6–7; TC10,12. |
| T6 | New editor `Command`s (apply/revert) | T5 | med | `tools/path_planner/editor/commands.py` | `SetEdgeOverride` / `ClearEdgeOverride` / `SetPipelineMode` / `SetFieldWideFlag` / `SetFieldDimensions`, following the `SetResetSpec` apply/revert + captured-original pattern; mutate `FieldModel.edges` keyed by `(type,u,v)` and the pipeline/flag/dim fields. Satisfies AC8,10; TC13–14. |
| T7 | Inspector panels: EDGE OVERRIDES list + PIPELINE DEFAULTS + toggles + FIELD_MM | T5 | med | `tools/path_planner/editor/panels.py` | Build the base-C `EDGE OVERRIDES` collapsible list (mirror the Pivots list: rows + per-row edit/delete + add-row inline editor; `type` auto-derived & read-only; `primitive_mode` combobox values context-switch on edge type; `direction`/`stop_at_line` comboboxes; geometry not shown). New `PIPELINE DEFAULTS` collapsible (4 mode comboboxes). Extend GLOBAL KNOBS with `blind_projection_auto_seek` checkbutton + `FIELD_MM` width/height entries. Use house widgets/styles; expose callback slots. **Own the graft's panel side: a `locate_override_row(edge_type,u,v)` method that scrolls to + highlights an existing row, and a `prefill_add_row(edge_type,u,v)` method** — both invoked by T8. Implements the `## UI/UX Specification`. Satisfies AC8,10. |
| T8 | App wiring: selection→inspector, edge-mode click graft, pure-JSON save, retire `_pending_renames` | T5, T6, T7 | high | `tools/path_planner/editor/app.py` | Wire panel callbacks → dispatch the new commands; refresh inspector from model (edges + pipeline + flag + FIELD_MM). Graft: reuse existing Edge mode `_edge_selection` so clicking an existing edge calls `panels.locate_override_row(...)` if a row exists, else `panels.prefill_add_row(...)` from the clicked segment (panel owns the scroll/highlight; app owns the wiring). **Remove the entire `_pending_renames` rename→Python-EDGES-flush subsystem in `app.py` — the actual `ast_rewrite` call sites (NOT in migrate.py): the `_save()` flush (lines ~2593-2595), `_record_pending_rename` (~2513), and the EDGES-row finder (~2506).** Save becomes pure-JSON; confirm a node rename now updates EDGES `u`/`v` through the model (the `RenameNode` command must cascade into `model.edges`). Satisfies AC8,9,11; TC16–17. |
| T9 | Retire `ast_rewrite.py` (+ its two tests) | T8 | low | `tools/path_planner/editor/ast_rewrite.py` (delete), `tools/path_planner/tests/test_editor_ast_rewrite.py` (delete), `tools/path_planner/tests/test_editor_app_renames.py` (delete or rewrite) | Delete `ast_rewrite.py` and `test_editor_ast_rewrite.py`. `test_editor_app_renames.py` imports `ast_rewrite` and monkeypatches `rewrite_edges_uv` to assert the old Save flush — it will fail collection once `ast_rewrite` is gone; **delete it (its rename→EDGES-in-model behavior is re-covered by TC17) or rewrite it to assert the new model-cascade path**. Confirm no remaining importers of `ast_rewrite` (migrate.py has none — its EDGES-tail change is in T2). Satisfies AC11; TC18. |
| T10 | Python data-layer tests + golden check | T1, T2, T3 | low | `tools/path_planner/tests/test_field_data.py`, `test_editor_loader.py`, `test_header_writer.py`, `test_build_routes.py`, `test_graph_builder*.py`, `test_editor_migrate.py` | Add TC1–12,19,20,22. Assert string-tag + enum-instance primitive_mode; explicit string-`"DEFAULT"` rejection; optional geometry derivation (incl. TC20 pivot-leg path through `_build_authored_graph`); pipeline-key validation + field-sourced emission (present → custom, absent → default + TC8 golden re-run zero-diff); loader/round-trip incl. a geometry-carrying entry (verbatim) + absent-stays-absent; `to_module_proxy` AND `to_json_dict` parity; build_routes freshness; `--migrate` carries the Python EDGES literal into JSON. Verify `graph_builder` needs no change. Satisfies AC1–7,12,14. |
| T11 | Editor GUI tests | T6, T7, T8 | low | `tools/path_planner/tests/test_editor_commands.py`, `test_editor_panels_smoke.py`, `test_editor_inspector.py` | Add TC13–16,21: command apply/revert round-trips; panels smoke (new sections + callback slots + `locate_override_row`/`prefill_add_row` survive); app inspector behavior (edge-mode click locates/pre-fills row; refresh shows overrides + pipeline + toggle + dims); TC21 node-rename cascades into `model.edges` (replaces the deleted `test_editor_app_renames.py` assertions). Satisfies AC8–11. |
| T12 | ADR-047 + docs | — | low | `docs/decisions/047-json-edges-overrides-and-pipeline-config.md`, `DECISIONS.md`, `docs/decisions/022-field-data-json-loader.md`, `tools/path_planner/README.md` | Write ADR-047 (JSON string-tag encoding for EDGES overrides; `EDGES`/`PIPELINE_CONFIG` field-sourced via JSON; geometry-optional overlay; editor owns EDGES; `ast_rewrite` retired). Mark `022-field-data-json-loader.md` status "Partially superseded by ADR-047" + cross-link. Update `DECISIONS.md` index + status table. Update `README.md` editor usage (override list, pipeline defaults) **and remove the `editor/ast_rewrite.py` row at README line ~293 (and the LibCST rename-cascade mention) since the module is deleted**. Satisfies AC13. |

Parallelizable start set: **T1, T2, T5, T12** (no blockers). Then T3 (after T2); T6, T7 (after T5);
T8 (after T5,T6,T7); T9 (after T8); T10 (after T1,T2,T3); T11 (after T6,T7,T8).

## Notes for Implementer

- **`graph_builder` and `header_writer` already handle string `primitive_mode`** — `graph_builder`
  forwards it verbatim (`_extract_bidirectional_attrs`) and `header_writer._override_enum_value`
  resolves a name (`enum_cls[mode_tag]`). The only blocker today is the validator. Do NOT add a second
  resolution path.
- **Dual-serialization is the #1 risk** (both critics): a field that lands in `to_json_dict` but NOT
  `to_module_proxy` (or vice versa) is persisted-but-unvalidated. T5 must touch BOTH, and TC12 must
  prove parity. Mirror `reset_nodes`/`blind_projection_auto_seek` exactly.
- **Keep `field_data`/`header_writer` reads of the new keys non-strict** (`getattr(..., default)` /
  `_data.get(..., default)`) so the many `SimpleNamespace` fixtures (which omit them) keep working —
  this avoids the strict-emitter-field-fixtures blast radius.
- **Confirm the production `field_data.json` has no `EDGES` / `pipeline_*` keys before relying on
  "routes byte-identical"** (AC12). The production field carries no overrides today; if it somehow
  does, regenerate + commit `routes.{h,cpp}` with the change. (The `field_data_stop_at_line.json`
  *test fixture* DOES carry a geometry-bearing EDGES entry — that's intentional backward-compat and is
  loaded via a `SimpleNamespace`, not the editor `FieldModel`; pin its behavior in TC2/round-trip.)
- **Backward-compat for geometry-bearing entries:** a hand-authored EDGES entry that DOES carry
  `distance_cm`/`heading_deg` still validates and is stored/emitted verbatim by `FieldModel` (no
  normalization). Only the editor's *newly authored* overrides are override-only.
- **Goldens:** do NOT add the new keys to the toy field — keep `routes_toy.{h,cpp}` byte-identical
  (absent keys → default emission). Cover field-sourced pipeline emission + string-tag overrides via
  dedicated tests building a small field, not via the toy golden. If you nonetheless add a deliberate
  exemplar, rebaseline via `python tools/path_planner/tests/test_header_writer.py` and commit the
  golden with the change (CI runs a routes-freshness check).
- **EDGES entry identity is `(type, u, v)`** — any add/remove/dedup in commands or the model must key
  on all three (a directed pair is not unique across types).
- **`direction`/`stop_at_line` are forward-only** (`u→v`). The GUI authors per directed edge; to
  override the reverse, the operator adds a second row with `u`/`v` swapped. Surface this in the row
  summary (e.g. show the directed `u→v`), don't silently imply symmetry.
- **ADR number:** 047 is the next free number. There is a pre-existing ADR-022 *collision* on `main`
  (`022-dual-motor-profile.md` and `022-field-data-json-loader.md`); do NOT renumber either — only
  edit `022-field-data-json-loader.md`'s status line.
- **Run the FULL pytest AND the full host ctest** after T1/T2/T3 — a validator/field-contract change
  can break fixtures across both layers (and the inline toy fixture in `test_header_writer.py`).
- **Rollback:** the change is Python-tooling only and additive; reverting the branch restores the
  prior editor + planner with no firmware/routes impact. `ast_rewrite.py` deletion (T9) is the only
  removal — recoverable from git if a legacy Python-EDGES authoring need resurfaces.
