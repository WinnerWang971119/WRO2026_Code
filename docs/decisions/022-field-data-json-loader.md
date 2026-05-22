# ADR-022: field_data geometry lives in JSON; field_data.py is a loader

**Status:** Implemented
**Date decided:** 2026-05 (Issue #10 Phase 2)

## Rule

Geometry input (NODES, NODE_TYPES, LINE_SEGMENTS, BLIND_EDGES, OBSTACLES, FIELD_MM, MAP_PNG, ROBOT_RADIUS_CM, COST_BLIND_MULTIPLIER, START_NODE, START_HEADING_DEG) lives in `tools/path_planner/field_data.json`. The Python module `tools/path_planner/field_data.py` is a thin loader that re-exposes those values as module attributes via `load_field_json()`. The rich `EDGES` table (with `Pivot` / `LineMode` / `TurnMode` enum imports) stays in Python because JSON has no enum type.

## Why

Scope 2 of [Issue #10](https://github.com/WinnerWang971119/WRO2026_Code/issues/10) ships a Tk-based live editor that authors the field graph by mouse. The editor needs a serialisation target that is (a) trivial to write deterministically from a GUI, (b) does not require Python-source rewriting for the bulk of edits, and (c) survives byte-identical idempotent dumps. JSON via [editor/loader.py](../../tools/path_planner/editor/loader.py) gives us all three.

The `EDGES` table stays in Python because per-edge `primitive_mode` / `pre_turn_mode` / `pivot` overrides reference Python enum values — JSON would require a name-to-enum mapping layer that defeats the simplicity gain. Node renames cascade into the `EDGES` table via LibCST (see the editor README) so comments and per-entry formatting survive intact.

## How to apply

- All GUI-managed edits (every NODES / edge / obstacle / start mutation) go through `FieldModel` → `loader.dump_field_json` → `field_data.json`. Hand-editing the JSON is still permitted and re-runs through the same loader on next `build_routes`.
- The rich `EDGES` table is hand-edited in `field_data.py`. When a node is renamed in the GUI, the editor's Save handler invokes `editor.ast_rewrite.rewrite_edges_uv(field_data.py, renames)` to update `"u"` / `"v"` string literals — every other token (including comments) is preserved.
- `PIPELINE_CONFIG` defaults still live in `tools/path_planner/pipeline_config.py` and are out of scope for the GUI.
- One-shot migration from the legacy Python-literal layout: `uv run python tools/path_planner/edit_field.py --migrate`.

## Trigger to revisit

- If a future feature requires the GUI to author rich `EDGES` overrides, the JSON schema must absorb a string-keyed enum representation (e.g. `"primitive_mode": "FAST"`) and the loader must translate strings back to enum members. At that point the Python-side EDGES literal should be deleted entirely.
- If the LibCST rewriter cannot keep up with future EDGES schema additions, fall back to: GUI refuses to rename a node referenced by EDGES; the user hand-edits Python first.

## See also

- [Issue #10](https://github.com/WinnerWang971119/WRO2026_Code/issues/10)
- [docs/plans/2026-05-20-issue-10-live-field-editor.md](../plans/2026-05-20-issue-10-live-field-editor.md)
- [tools/path_planner/editor/loader.py](../../tools/path_planner/editor/loader.py) — JSON load + dump.
- [tools/path_planner/editor/migrate.py](../../tools/path_planner/editor/migrate.py) — one-shot Python→JSON port.
- [tools/path_planner/editor/ast_rewrite.py](../../tools/path_planner/editor/ast_rewrite.py) — LibCST EDGES rename cascade.
- [tools/path_planner/README.md](../../tools/path_planner/README.md) — editor usage guide.
