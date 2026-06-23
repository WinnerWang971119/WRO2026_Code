# Editor reset-node auto-fill: derive the wall AXIS from geometry

**Goal:** Fix the field editor's reset-node auto-fill so it picks the wall *axis* from the node's position (the nearest wall) instead of always defaulting to `x`, and warns when the operator placed the node too close to that wall — so a reset node placed against a top/bottom wall no longer silently re-anchors against a side wall.

**Approach:** Add an axis-auto helper `reset_autofill_auto(node_xy, field_mm)` that evaluates all four field walls and returns the nearest wall's `(axis, wall_coord, facing)`. Change the existing single-axis `reset_autofill` to also pick the *nearest* wall on its axis (no longer skipping a too-close wall for a far qualifying one), so the manual axis-change path behaves consistently. Wire both auto-derive seed paths (create + convert) onto the auto helper, and surface an explicit "too close — move it back" status warning whenever any auto-derived spec seeds a wall under the 14 cm pivot-clearance floor (the Save-time validator stays the hard gate). The facing/wall_coord authoring contract (ADR-043) is unchanged — only how the editor *suggests* the axis changes.

## Scope

- **In scope:**
  - New pure helper `projection_edges.reset_autofill_auto(node_xy, field_mm) -> (axis, wall_coord, facing)` picking the nearest of all four walls.
  - New pure predicate `projection_edges.is_reset_wall_too_close(node_xy, axis, wall_coord) -> bool` (standoff `< WALL_PIVOT_CLEARANCE_CM`).
  - Change `projection_edges.reset_autofill(node_xy, axis, field_mm)` selection from "nearest qualifying wall, else larger standoff" to **"nearest wall on the axis"** (tie → low/`0` wall).
  - New `commands.SetResetSpec.autofilled_auto(name, node_xy, field_mm)` classmethod.
  - `app.py`: the two auto-derive seed paths (`_on_panel_node_type_change` reset branch, `_add_reset_node`) use the auto helper instead of the hardcoded `axis="x"`; a shared `_warn_if_reset_too_close` helper fires on the two seed paths **and** the `_on_panel_reset_spec_change` axis-changed branch.
  - Tests: auto-axis selection, tie-break, too-close warning (seed + manual-axis paths), the new predicate; rewrite `test_convert_to_reset_autofills_wall_and_facing`.
- **Out of scope:**
  - Any edit to `tools/path_planner/field_data.json` — the already-saved wrong specs (`reser2`, `test`) are NOT auto-corrected; the operator fixes them by flipping the axis combobox (the existing recompute re-derives wall+facing). No routes regeneration.
  - Any change to `validator.py`, `graph_builder.py`, `compiler.py`, `header_writer.py`, the sim, or firmware. The facing/wall_coord contract and ADR-043 are untouched.
  - A canvas facing-arrow / visual orientation cue (considered during debug; the user chose the data fix, not a canvas change).
  - The axis-**unchanged** hand-edit branch of `_on_panel_reset_spec_change` (operator typing a wall/facing by hand is not "auto-derived"; the existing `_revalidate` badge already flags a too-close hand-edit).
  - A "Suggest from position" button — the user chose "flip the axis combobox" for fixing existing nodes.

## Decisions

- **Auto-axis rule = nearest wall overall.** `reset_autofill_auto` evaluates all four walls (x=0 → facing 0, x=field_w → facing 180, y=0 → facing 90, y=field_h → facing 270) and returns the candidate with the smallest standoff `abs(wall - node_coord)`. — The nearest wall is the one the operator placed the node against; this is what was silently wrong before.
- **Seed the nearest wall even when it is too close (< 14 cm).** Do not skip it for a far valid wall. — A too-close near wall means the operator aimed at that wall and placed it too close; seeding it + warning is more truthful than silently re-anchoring 3 m away. Confirmed with the user.
- **Tie-break: prefer `y`, then the low (`0`) wall.** When two candidates have exactly equal standoff, choose `y` over `x`; within an axis choose wall `0` over the far wall (keeps `reset_autofill`'s existing "low wall first" convention). — User-chosen.
- **Warn on every auto-derived path, not just create/convert.** The manual axis-change branch of `_on_panel_reset_spec_change` re-derives via `reset_autofill`, so it must seed the nearest-on-axis wall and warn too. — User-chosen ("warn everywhere a spec is auto-derived").
- **Warning = explicit status message + Invalid badge; the spec is still applied.** The too-close spec is dispatched (the node exists, just invalid), an inline status message names the standoff and the fix, the validation badge goes Invalid, and `model.validate()` at Save is the authoritative hard reject. — Matches the editor's "auto-fill suggests, never traps" rule and the existing inline-feedback pattern.
- **Explicit helper needed despite `_revalidate`.** `_dispatch` → `_refresh_view` → `_revalidate` already runs the full validator and flips the badge in the live app, but the headless app tests stub `_refresh_view`, so an explicit `_warn_if_reset_too_close` (status + badge) is required for the warning to be observable and is harmless belt-and-suspenders in the app.
- **Keep `reset_autofill` (single-axis) and add the auto variant — don't replace.** The manual axis-change path still calls the single-axis `reset_autofill` for the operator's explicitly chosen axis; only its wall-*selection* changes (nearest, not nearest-qualifying). The manual override (combobox) is otherwise preserved.

## Acceptance Criteria

- [ ] **AC1** — Converting a plain node to `reset` (inspector type combobox) seeds the spec against the **nearest of all four walls**, not always `x`. For `reser2`-like geometry (node near a y-wall) the seeded `axis` is `"y"`.
- [ ] **AC2** — Add-node → `reset` (canvas) seeds the same nearest-wall spec as the convert path for the same coordinates.
- [ ] **AC3** — The seeded `facing` points away from the chosen wall (0/180 for x=0/x=field_w; 90/270 for y=0/y=field_h), so the existing `_validate_reset_nodes` facing check passes for any wall ≥ 14 cm.
- [ ] **AC4** — When the nearest (seeded) wall is `< WALL_PIVOT_CLEARANCE_CM` (14 cm), the spec is still applied, an inline status message names the actual standoff and the `WALL_PIVOT_CLEARANCE_CM` value and tells the operator to move it back, and the validation badge is Invalid. The message is ASCII-only and contains the numeric standoff.
- [ ] **AC5** — When the seeded wall is ≥ 14 cm, no too-close warning is emitted (no too-close status message; the warning helper does not force the badge Invalid).
- [ ] **AC6a** — Flipping the axis combobox to a different axis re-derives the spec against the **nearest wall on the chosen axis** (not nearest-qualifying). For `reser2` (stored axis x, wall 320), selecting `y` yields `axis="y", wall_coord=0, facing=90`. Its y=0 standoff is 19.6 cm (≥ 14), so this fix fires **no** too-close warning.
- [ ] **AC6b** — On the axis-change path, when the chosen axis's nearest wall is `< 14 cm`, the same too-close warning fires (verified on a deliberately wall-hugging geometry, not on `reser2`).
- [ ] **AC7** — Equal-standoff ties resolve to `y` (across axes) and to wall `0` (within an axis).
- [ ] **AC8** — `field_data.json` is unchanged by this work; `reser2`/`test` keep their stored specs.
- [ ] **AC9** — The full planner pytest suite is green: `uv run --extra dev pytest tools/path_planner/tests/`, with **one intended change** — `test_convert_to_reset_autofills_wall_and_facing` is rewritten (T4) for the new auto-axis result. No *other* existing editor/validator/header-writer test regresses.

## Contracts & Interfaces

Single source of truth for every seam more than one task touches. Owners produce; consumers conform byte-for-byte.

### Signatures

- `projection_edges.reset_autofill_auto(node_xy: tuple[float, float], field_mm: tuple[float, float]) -> tuple[str, float, float]`
  Returns `(axis, wall_coord, facing)` for the nearest of the four walls. `axis` ∈ `{"x","y"}`; `facing` ∈ `[0, 360)`. Tie-break: prefer `y`, then low wall. **Never raises** on a too-close wall (it returns the best-effort nearest spec). — owner: **T1**; consumers: **T2, T4**.
- `projection_edges.reset_autofill(node_xy: tuple[float, float], axis: str, field_mm: tuple[float, float]) -> tuple[float, float]`
  Returns `(wall_coord, facing)` for the **nearest wall on `axis`** (tie → low/`0` wall). Raises `ValueError` on an axis not in `{"x","y"}` (unchanged). Selection logic changed; signature unchanged. — owner: **T1**; consumers: **T3 (axis-change path), T4**.
- `projection_edges.is_reset_wall_too_close(node_xy: tuple[float, float], axis: str, wall_coord: float) -> bool`
  `True` iff `abs(wall_coord - (node_xy[0] if axis == "x" else node_xy[1])) < WALL_PIVOT_CLEARANCE_CM`. Single-sources the standoff check against `validator.WALL_PIVOT_CLEARANCE_CM` (already imported in `projection_edges`). — owner: **T1**; consumers: **T3, T4**.
- `commands.SetResetSpec.autofilled_auto(name: str, node_xy: tuple[float, float], field_mm: tuple[float, float]) -> SetResetSpec`
  Builds a `SetResetSpec` whose `axis`/`wall_coord`/`facing` come from `reset_autofill_auto`. Mirrors the existing `autofilled` classmethod but derives the axis. — owner: **T2**; consumers: **T3**.
- `EditorApp._warn_if_reset_too_close(self, name: str, axis: str, wall_coord: float) -> None`
  After a successful auto-derive dispatch: if `is_reset_wall_too_close(self._model.nodes[name], axis, wall_coord)`, push an inline status message (ASCII-only) naming the numeric standoff and the `WALL_PIVOT_CLEARANCE_CM` fix, and call `self._panels.set_validation_badge(False)`. No-op when the wall clears the floor. — owner: **T3**; internal.

**App-supplied inputs (pin these so the seam is explicit):** both seed paths pass `node_xy=self._model.nodes[name]` and `field_mm=self._model.field_mm` to `autofilled_auto`. How T3 learns the auto-derived axis for the warning: after the dispatch succeeds, **read it back** from `self._model.reset_nodes[name]` (`["axis"]`, `["wall_coord"]`) — `SetResetSpec.apply` has written it — then call `_warn_if_reset_too_close`. The axis-change path already holds `axis`/`new_wall` locally and passes them directly.

**Badge writers (reconcile the two methods):** in the live app, `_dispatch → _refresh_view → _revalidate` calls `SidePanels.set_validation(ok, msg)` (running the full `model.validate()`, which fails on a < 14 cm node and reports Invalid with the validator's own message). The warning helper additionally calls `SidePanels.set_validation_badge(False)`. T3 must confirm these two `SidePanels` methods drive the **same** badge widget and do not double-toggle to a wrong state; if `set_validation` already reports Invalid in the app, the helper's `set_validation_badge(False)` is consistent (both Invalid) and is what the **headless tests** observe (where `_refresh_view`/`_revalidate` are stubbed, so the helper is the only badge writer). Do not call `set_validation` from the helper (the `_PanelsSpy` does not implement it).

### Shared constant

- `WALL_PIVOT_CLEARANCE_CM` — sourced from `validator.py`, already re-exported in `projection_edges.__all__`. T1 uses it in `is_reset_wall_too_close`; T3 imports it (via `projection_edges`) for the warning message text. Do not redefine the literal `14`.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `tools/path_planner/editor/projection_edges.py` | T1 | — |
| `tools/path_planner/editor/commands.py` | T2 | — |
| `tools/path_planner/editor/app.py` | T3 | — |
| `tools/path_planner/tests/test_editor_reset_autofill.py` | T4 | — |

### Naming

- Classmethod name is exactly `autofilled_auto` (parallels `autofilled`).
- Helper name is exactly `reset_autofill_auto` and `is_reset_wall_too_close`.
- Facing values per wall (must match `reset_autofill`): x=0 → 0.0, x=field_w → 180.0, y=0 → 90.0, y=field_h → 270.0.

## Data / geometry notes

Field: origin SW corner, X east, Y north, centimetres, headings CCW from +x in `[0, 360)`. `field_mm = (width_mm, height_mm)`; cm = mm / 10. The four wall candidates for a node `(x, y)`:

```
axis "x": wall 0        standoff |0 - x|          facing 0.0    (front +x, away from west wall)
axis "x": wall field_w  standoff |field_w - x|    facing 180.0  (front -x, away from east wall)
axis "y": wall 0        standoff |0 - y|          facing 90.0   (front +y, away from south wall)
axis "y": wall field_h  standoff |field_h - y|    facing 270.0  (front -y, away from north wall)
```

`reset_autofill_auto` = the min-standoff candidate over all four, tie-broken `(standoff, axis_rank, wall_rank)` with `axis_rank: y=0, x=1` (prefer y) and `wall_rank: 0=0, far=1` (prefer low wall). `reset_autofill(axis)` = the min-standoff of the two candidates on that axis, same low-wall tie-break.

Worked check (field 320×154):
- `reser2` (238.8, 19.6): standoffs x0=238.8, x320=81.2, y0=19.6, y154=134.4 → nearest **y=0 (19.6)**, ≥14 → `("y", 0.0, 90.0)`, no warning.
- `test` (309.4, 69.1): standoffs x0=309.4, x320=10.6, y0=69.1, y154=84.9 → nearest **x=320 (10.6)**, < 14 → `("x", 320.0, 180.0)` **with** too-close warning (was silently `("x", 0.0, 0.0)` 309 cm away).
- convert-test node B (200, 10) in 300×200: standoffs x0=200, x300=100, y0=10, y200=190 → nearest **y=0 (10)**, < 14 → `("y", 0.0, 90.0)` with too-close warning.

## Error Handling

- **Node too close to the seeded wall (< 14 cm):** spec is applied; `_warn_if_reset_too_close` sets a specific status message + Invalid badge; `model.validate()` at Save hard-rejects with the existing `_validate_reset_nodes` clearance message. The operator moves the node or switches walls.
- **Unknown axis passed to `reset_autofill`:** unchanged — raises `ValueError`, surfaced inline by the existing guard.
- **`_add_reset_node` first `AddNode` rejected (duplicate name):** unchanged — the second (auto-fill) dispatch is skipped; no half-formed reset node.
- **Warning message text:** ASCII only (e.g. "move it back to at least 14 cm") to avoid any source/console encoding surprise; the constant is interpolated from `WALL_PIVOT_CLEARANCE_CM`, not hardcoded.

## Testing Strategy

**Levels:** Unit (pure geometry helpers + headless app handlers via the existing `EditorApp.__new__` + `_PanelsSpy` pattern). No real Tk window.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `reset_autofill_auto` for nodes nearest each of the 4 walls | Unit | Returns the nearest wall's `(axis, wall, facing)`: near x=0 → `("x",0,0)`; near x=field_w → `("x",field_w,180)`; near y=0 → `("y",0,90)`; near y=field_h → `("y",field_h,270)`. |
| TC2 | `reset_autofill_auto` picks y-wall when nearer than any x-wall (`reser2` geometry) | Unit | `(238.8,19.6)` in 320×154 → `("y", 0.0, 90.0)`. |
| TC3 | `reset_autofill_auto` tie-break: equal x/y standoff | Unit | Concrete exact tie: node `(30.0, 30.0)` in a 200×200 cm field → nearest x-wall (x=0, standoff 30) and nearest y-wall (y=0, standoff 30) are exactly equal → axis `"y"`, wall 0, facing 90. Centered-on-axis tie (e.g. `(100.0, 30.0)` in 200×200, both x-walls standoff 100 but y=0 nearer at 30 — use a square-symmetric node to force the within-axis tie) → low wall `0`. Use exact integer coordinates so the tie is bit-exact, never a float near-miss that the standoff key would resolve first. |
| TC4 | `reset_autofill_auto` seeds a too-close wall (best-effort, no raise) | Unit | `test` geometry `(309.4,69.1)` in 320×154 → `("x",320.0,180.0)`, and `is_reset_wall_too_close` is `True` for it. |
| TC5 | `is_reset_wall_too_close` boundary | Unit | `True` for standoff `< 14`; `False` at exactly `14` and above; correct per-axis coordinate selection. |
| TC6 | `reset_autofill` (single axis) now picks nearest, not nearest-qualifying | Unit | For a node whose near wall on the axis is < 14 and far wall ≥ 14, returns the **near** wall (changed behavior), tie → wall 0. The existing TC5-style cases (near wall ≥ 14) are unchanged. |
| TC7 | Convert plain → reset auto-derives the axis (rewrite of existing test) | Unit | `_on_panel_node_type_change("B","reset")` on B (200,10)/300×200 → `reset_nodes["B"] == {"axis":"y","wall_coord":0.0,"facing":90.0}`; a too-close status message is recorded and the badge is Invalid; `can_undo` is True. |
| TC8 | Add-node reset auto-derives the axis | Unit | `_add_reset_node("R",(200,10))` → same nearest-wall spec as TC7 for those coordinates. |
| TC9 | Seed warning fires only when too close | Unit | A node ≥ 14 cm from its nearest wall → spec applied; assert the recorded `status_messages` contain **no** too-close substring (check the specific message text, not just the badge) and the helper did not append `False` to `validation_badges`. |
| TC10 | Manual axis-change re-derives nearest-on-axis + warns | Unit | Drive `_on_panel_reset_spec_change(name, <other axis>, …)` on a model whose node is < 14 cm from the chosen axis's nearest wall → a too-close status message is recorded and the dispatched spec matches `reset_autofill` for that axis. Separately assert the `reser2`-style fix (flip to `y`, standoff 19.6 ≥ 14) records **no** too-close message. |
| TC11 | Axis-unchanged hand-edit is unaffected | Unit | Existing `test_tc6_axis_unchanged_*` behavior preserved — no too-close warning injected on a verbatim hand-edit dispatch; out-of-range facing still inline-rejected. |
| TC12 | Too-close warning message content | Unit | The recorded too-close message contains the numeric standoff (e.g. for `test` geometry, `"10.6"`) and the clearance value (`"14"`), and `message.isascii()` is `True`. |

**Test data:** reuse the headless patterns already in `test_editor_reset_autofill.py` — `matplotlib.use("Agg")` + no-op the import-time `use("TkAgg")`; `EditorApp.__new__` + injected `_history`/`_model`/`_PanelsSpy`; `_refresh_view`/`_mark_dirty` stubbed. The `_PanelsSpy` already records `status_messages` and `validation_badges` — assert the too-close message and `validation_badges[-1] is False`. Pure-helper tests call `projection_edges` directly with a `field_mm` tuple.

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/` (FULL suite — never just the one file; a reset-autofill/fixture change can redden siblings). Optionally run the host C++ ctest as well if any shared contract is touched (it is not here, but the user's gotcha rules call for the full pytest at minimum).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Geometry helpers: auto-axis + too-close predicate + nearest-on-axis selection | — | med | `tools/path_planner/editor/projection_edges.py` | Add `reset_autofill_auto(node_xy, field_mm) -> (axis, wall_coord, facing)` evaluating all four walls and returning the smallest-standoff candidate, tie-broken `(standoff, axis_rank[y<x], wall_rank[0<far])`. Add `is_reset_wall_too_close(node_xy, axis, wall_coord) -> bool` comparing the standoff to `WALL_PIVOT_CLEARANCE_CM`. Change `reset_autofill(node_xy, axis, field_mm)` to pick the **nearest** of the two walls on `axis` (tie → wall 0), dropping the qualifying-filter / larger-standoff fallback; keep the `ValueError` on bad axis and the facing-per-wall mapping. Update docstrings + `__all__`. Match the module's existing comment density. Satisfies AC1–AC3, AC6, AC7. |
| T2 | `SetResetSpec.autofilled_auto` classmethod | T1 | low | `tools/path_planner/editor/commands.py` | Add a classmethod `autofilled_auto(cls, name, node_xy, field_mm) -> SetResetSpec` mirroring `autofilled` but calling `projection_edges.reset_autofill_auto` (no `axis` arg). Leave `autofilled`, `apply`, and `revert` unchanged — the atomic type+spec invariant is preserved. Doxygen/docstring matching the existing class style. Satisfies AC1, AC2. |
| T3 | Wire auto-axis into the seed paths + too-close warning helper | T1, T2 | med | `tools/path_planner/editor/app.py` | Add `_warn_if_reset_too_close(self, name, axis, wall_coord)` per the contract (status message naming the standoff + `WALL_PIVOT_CLEARANCE_CM`-derived fix text, ASCII only; `set_validation_badge(False)`; no-op when not too close). In `_on_panel_node_type_change` reset branch (≈1066-1081) and `_add_reset_node` (≈1778-1812): replace `SetResetSpec.autofilled(..., axis=_RESET_SPEC_DEFAULT_AXIS, ...)` with `SetResetSpec.autofilled_auto(name, node_xy=model.nodes[name], field_mm=model.field_mm)`; **immediately after the successful seed `_dispatch` returns True** (in `_add_reset_node`, before `_select_node`/`_reveal_reset_spec_section`), read back the seeded `axis`/`wall_coord` from `model.reset_nodes[name]` and call `_warn_if_reset_too_close`. In `_on_panel_reset_spec_change` axis-**changed** branch (≈1262-1272): after the existing `reset_autofill` dispatch succeeds, call `_warn_if_reset_too_close(name, axis, new_wall)`. Do NOT touch the axis-unchanged hand-edit branch. `_RESET_SPEC_DEFAULT_AXIS` may stay (unused) or be removed if no longer referenced. Ensure `WALL_PIVOT_CLEARANCE_CM` is importable in app.py (via `projection_edges`). Confirm `SidePanels.set_validation` and `set_validation_badge` drive the same badge and don't double-toggle (Contracts §"Badge writers"). Satisfies AC1, AC2, AC4–AC6b. |
| T4 | Tests: auto-axis, tie-break, too-close warning, predicate; rewrite convert test | T1, T2, T3 | low | `tools/path_planner/tests/test_editor_reset_autofill.py` | Implement TC1–TC12. **Rewrite** `test_convert_to_reset_autofills_wall_and_facing` (including its now-wrong comment block) to expect the auto-axis result for B (200,10) → `("y",0,90)` plus the too-close warning (standoff 10 < 14: a too-close `status_messages` entry recorded and `validation_badges[-1] is False`), and `can_undo True`. Add pure-helper tests for `reset_autofill_auto` (4 walls + reser2/test geometries + the exact-tie coordinate) and `is_reset_wall_too_close` (boundary at exactly 14). Add app-handler tests for the convert + add-node + manual-axis-change too-close warnings via the `_PanelsSpy` (`status_messages`, `validation_badges`). **Spy precondition:** the headless app tests must keep `_refresh_view` stubbed (as `_make_app` does) so the convert/add `_dispatch` does not reach the real `_revalidate`→`set_validation` (the `_PanelsSpy` implements only `set_status`/`refresh`/`set_validation_badge`, NOT `set_validation` — an unstubbed path would `AttributeError`). Keep `test_tc5_*`/`test_tc6_*`/`test_tc8_*` passing: the `test_tc5_*` away-from-wall cases have nearest wall ≥14 so "nearest" == "nearest qualifying" → unchanged; the wall-hugging best-effort test resolves to wall 0 under both old `max`-on-equal-standoff and new `min`-with-low-wall-first, and its asserts are lenient → unchanged. Run the FULL suite. Satisfies AC9 + verifies AC1–AC8. |

**Parallelism:** T1 starts immediately. T2 waits on T1 (calls the new helper). T3 waits on T1+T2. T4 waits on T1+T2+T3. Each task owns a distinct file — no two tasks edit the same file, so no merge conflicts.

## Notes for Implementer

- **Do NOT touch `field_data.json`, routes, the validator, the sim, or firmware.** This is editor-suggestion-only. ADR-043's facing/wall_coord contract is unchanged — the validator at `validator.py:1243` is the authority and already rejects a facing toward the wall and a < 14 cm standoff.
- **The warning is observable in tests because `_refresh_view` is stubbed there.** In the live app, `_dispatch → _refresh_view → _revalidate` also runs `model.validate()` and flips the badge with the validator's own message; the explicit `_warn_if_reset_too_close` is required for the headless tests and is consistent (both set Invalid) in the app. Call the helper AFTER `_dispatch` returns so it runs after `_revalidate`.
- **`_PanelsSpy` already records `status_messages` and `validation_badges`** (see the existing `test_editor_reset_autofill.py`); the warning helper must use `self._set_status(...)` (→ `set_status`) and `self._panels.set_validation_badge(False)` so the spy captures both. Do not call `set_validation(ok, msg)` from the helper (the spy doesn't implement it).
- **Existing `test_tc5_*` cases do not change** — their nodes sit ≥ 14 cm from the nearest wall, so "nearest" equals "nearest qualifying". The wall-hugging best-effort test (`test_tc5_wall_hugging_node_returns_best_effort_spec`) is **unaffected**: the node is symmetric (equal standoff to both walls on the axis), so the old `max(..., key=standoff)` and the new `min`-with-low-wall-first both resolve to wall `0`, and its asserts are lenient (`wall in (0.0, field_w)`, `abs(wall-coord) < 14`). State this definitively in the test rationale; do not leave it as "likely passes."
- **`set_validation` vs `set_validation_badge` are two `SidePanels` methods.** `_revalidate` (live app only) calls `set_validation(ok, msg)`; the warning helper calls `set_validation_badge(False)`. Before relying on the helper, open `panels.py` and confirm both methods target the same badge widget so they cannot fight (e.g. helper sets Invalid then a later `set_validation(True)` flips it back). They should agree because a < 14 cm node fails `model.validate()`, so `_revalidate` also reports Invalid. The helper exists for the headless tests (where `_revalidate` is stubbed away) and is consistent in the app.
- **Tie-break implementation:** build the four candidates and `min(..., key=lambda c: (standoff, axis_rank, wall_rank))` with `axis_rank` mapping `y→0, x→1` and `wall_rank` mapping the low wall `→0`. This yields "prefer y, then low wall" deterministically without floating-point ordering surprises (use exact equality only via the sort key, never an `==` float compare for selection).
- **Keep `reset_autofill` raising `ValueError` on a bad axis** — the app's axis-change guard relies on it (defensive branch at `app.py:1267-1270`).
- **Run the FULL pytest suite**, not just the new file: `uv run --extra dev pytest tools/path_planner/tests/` (bare `uv run pytest` fails — pytest lives in the dev extra).
