# PIPELINE_* Field-Sourcing Audit Plan (issue #148)

**Goal:** Make every `PIPELINE_*` resolution site in `tools/path_planner/` read the field-sourced
`PipelineConfig.from_field_data(field_data)` instead of the module singleton `PIPELINE_CONFIG`, and
add a lint guard so a fourth drifted surface cannot land silently.

**Approach:** Surgical threading, not refactoring. The full audit found only **three** class-(b)
sites (`validator._regime`, `compiler.final_heading_after`, the `build_routes` diagnostic) —
everything in `sim/`, `ctps/`, `header_writer.py`, `sim/parity.py`, and `editor/` is already
correctly field-sourced. Fix those three in place, leaving the singleton alive only in the four
documented fallback roles the allowlist enumerates (`from_field_data`'s absent-key fallback, the
`sim/runtime.py` resolver default args, `_pass_through_heading_eligible`'s `None` fallback, and the
validator's two degrade closures), then add one new test file carrying (a) an end-to-end agreement
test across all four `PIPELINE_*` categories and (b) an AST-based lint that flags any new singleton
read outside that allowlist.

## Scope

- **In scope:**
  - `validator._validate_pivot_turns._regime` — resolve the blind seek rung against the edge's own
    `mode` and the field-sourced `cfg.blind` instead of `PIPELINE_CONFIG.blind` twice.
  - `compiler.final_heading_after` — accept an optional `field_data` and resolve
    `pipeline_default` from it, closing the documented-but-false "can never disagree with
    `compile_path`" claim.
  - `build_routes.py` diagnostic log — field-source it, print all four categories, move it after
    `validate()`.
  - A new test file `tools/path_planner/tests/test_pipeline_field_sourcing.py` holding the shared
    fixture, the cross-surface agreement test, and the anti-drift lint scan.
- **Out of scope:**
  - Changing what any `PIPELINE_*` default *means*, or the `PipelineConfig` dataclass itself.
  - Removing the `cfg: PipelineConfig = PIPELINE_CONFIG` default arguments from the `sim/runtime.py`
    resolvers — explicitly considered and rejected (see Decisions); ~20 test call sites depend on
    them and every production caller already threads a real `cfg`.
  - Any C++ / firmware change. `routes.h` / `routes.cpp` output is byte-unchanged by this work.
  - A new ADR. Diego chose the test guard over a doc rule.
  - Re-enabling `STOP_AT_LINE` as an authorable `mode` or `PIPELINE_BLIND_MODE` (ADR-064 removed it
    deliberately; this plan does not touch that).

## Decisions

- **Issue body is stale; fix anyway.** The issue is written against the pre-ADR-064 model: it names
  `primitive_mode` (retired, split into `speed` + `mode`) and its worked 12.4 cm-overshoot example
  needs `STOP_AT_LINE`, which `validator.py:1470` and `validator.py:2159` now both reject. Since
  rung 3 of `resolve_stop_at_line` is the only rung reading arg3/arg4, **the validator's seek
  verdict cannot change on any field that passes validation today** (and, given the `STOP_AT_LINE`
  degrade in the next Decision, cannot change mid-validation on a rejected one either). The
  validator fix is therefore
  behavior-preserving; its value is closing the arg3/arg4 contract so the invariant holds if
  `STOP_AT_LINE` (or any future BlindMode-driven seek) ever becomes authorable again. Fixed rather
  than closed as not-reproducible because it is built code with a wrong contract, per CLAUDE.md.
- **`final_heading_after` is the one live divergence.** `PIPELINE_DIRECTION_MODE: "FORCE_REVERSE"`
  is fully authorable, and `compile_path` (`compiler.py:360`, `cfg.direction`) would drive every hop
  REAR while `final_heading_after` (`compiler.py:186`, `PIPELINE_CONFIG.direction`) returns the
  FRONT-resolved heading — a silent 180° error. Latent only because the function has **no
  production callers** (10 test call sites; exported in `__all__` at `compiler.py:533`).
- **Validator degrades defensively rather than reordering `validate()`.** `_validate_pivot_turns`
  runs before `_validate_pipeline_config` (step 13), so a malformed `PIPELINE_BLIND_MODE` string
  would make a bare `from_field_data` raise `KeyError` from the pivot pass — losing the validator's
  precise-diagnostic contract (ADR-029). A new `_pipeline_cfg()` closure try/excepts down to the
  singleton, mirroring the `_pipeline_direction_default()` pattern already in the file. Reordering
  `validate()` was rejected: it changes which error surfaces first on a doubly-bad field and would
  churn existing error-ordering tests.
- **`_pipeline_cfg()` must ALSO degrade on `BlindMode.STOP_AT_LINE`, not just on exceptions.**
  `"STOP_AT_LINE"` is a *valid* `BlindMode` member name, so `from_field_data` returns it happily
  (`__post_init__` only rejects the `DEFAULT` sentinels) — the exception guard never fires. Without
  an explicit check, every non-overridden BLIND pivot leg would classify `"sensor"` during the pivot
  pass (step 10) and could trip a REAR-impossible / board-count rejection *before*
  `_validate_pipeline_config` (step 13) emits the precise ADR-064 message. That is the exact
  error-ordering hazard this fallback exists to prevent, and it would make TC8's asserted error
  identity depend on fixture geometry. So `_pipeline_cfg()` returns the singleton when the resolved
  `cfg.blind is BlindMode.STOP_AT_LINE`, with a comment naming step 13 as the real reporter.
- **`_pipeline_direction_default()` is left untouched.** It already field-sources correctly
  (`validator.py:708`) and `test_field_data.py:2352-2376` pins its FORCE_FORWARD field-sourcing
  fold. Note its malformed-name degrade-to-`DEFAULT` branch has **no** test today — that is a
  pre-existing gap, not one this plan creates, and T4 may pin it opportunistically while in the
  area. The new `_pipeline_cfg()` has whole-config degrade-to-singleton semantics and is used only
  for the blind axis. The asymmetry is deliberate and must be commented.
- **Rejected: a per-field `_pipeline_blind_default()` closure.** It would mirror
  `_pipeline_direction_default`'s shape exactly and avoid introducing a second differently-shaped
  fallback. Rejected because the issue explicitly asks for `PipelineConfig.from_field_data`, and a
  whole-config `cfg` is what `resolve_blind_mode(edge, cfg)` actually takes — a per-field helper
  would have to synthesize a `PipelineConfig` anyway.
- **`final_heading_after` takes an optional kwarg, not a required param.**
  `from_field_data(None)` already returns a config byte-identical to the singleton, so
  `field_data=None` keeps all 10 existing call sites byte-identical while making the fix available
  to the first production caller. Mirrors `compile_path`'s existing `field_data=None` tolerance.
- **Guard is a lint scan *and* an e2e test.** The e2e test proves the three fixes; the AST lint is
  what actually catches surface #4. Neither alone is sufficient.
- **Lint is AST-based, not regex.** `PIPELINE_CONFIG` appears in ~40 docstrings, comments, and
  error-message f-strings across `validator.py` alone. Only an AST walk distinguishes a real
  attribute read from prose.
- **Assumptions taken without asking (`--auto`):** (1) no ADR is written, per Diego's answer;
  (2) `routes.h`/`routes.cpp` need no regeneration because no emitter logic changes — but
  `build_routes.py` is still executed once as a smoke check, since its diagnostic is exactly the
  hidden consumer the `pipeline-config-rename-build-routes-diagnostic` gotcha warns about;
  (3) the host C++ ctest suite is not required, because the validator change adds no new rejection
  path and `routes.cpp` is unchanged — if any task *does* introduce a new rejection, ctest becomes
  mandatory (see Notes for Implementer).

## Acceptance Criteria

- [ ] **AC1** — `validator._regime` classifies a BLIND pivot leg's seek using
      `resolve_blind_mode(<edge data>, cfg)` as arg3 and `cfg.blind` as arg4, where `cfg` is
      field-sourced. No `PIPELINE_CONFIG.blind` read remains in `_regime` — verified behaviorally by
      TC5/TC6 and statically by TC9's whole-tree lint (`_regime` is not on the allowlist).
- [ ] **AC2** — A field authoring a malformed `PIPELINE_BLIND_MODE` (e.g. `"NONSENSE"`) still fails
      with the precise `FieldDataError` from `_validate_pipeline_config`, not a `KeyError` from the
      pivot pass. Every currently-passing validator test still passes unchanged.
- [ ] **AC3** — `final_heading_after(path, start_heading_deg, graph, field_data=<field>)` agrees with
      `compile_path(..., field_data=<field>)` on the final body heading when the field authors
      `PIPELINE_DIRECTION_MODE: "FORCE_REVERSE"`. Called with `field_data=None` (or omitted) it
      returns exactly what it returns today.
- [ ] **AC4** — `build_routes.py`'s `pipeline_config:` diagnostic prints all four categories
      (`speed`, `blind`, `turn`, `direction`) sourced from `PipelineConfig.from_field_data(field_data)`,
      and is emitted after `validate(field_data)` succeeds. The `# noqa: F401` `PIPELINE_CONFIG`
      import at `build_routes.py:69` is gone. `uv run python tools/path_planner/build_routes.py`
      still runs to completion and leaves `routes.h`/`routes.cpp` byte-unchanged.
- [ ] **AC5** — `tests/test_pipeline_field_sourcing.py` exercises all four `PIPELINE_*` categories
      set to non-default values through validator → compiler → emitted `routes.cpp` and asserts the
      surfaces agree.
- [ ] **AC6** — The lint scan fails when a `PIPELINE_CONFIG.<field>` attribute read is introduced in
      `tools/path_planner/` outside the allowlist, and passes on the post-fix tree. The failure
      message names the offending `file:line` and points at `PipelineConfig.from_field_data`.
- [ ] **AC7** — The full suite is green: `uv run --extra dev pytest tools/path_planner/tests/`.

## Data Model

No new persistent types. The existing enums are the vocabulary; the plan pins which **non-default**
member each category uses in the new fixture (default shown for contrast):

| Category | `field_data` key | Enum | Default | Fixture value (non-default) |
|---|---|---|---|---|
| speed | `PIPELINE_SPEED` | `SpeedTier` | `NORMAL` | `FAST` |
| blind | `PIPELINE_BLIND_MODE` | `BlindMode` | `SYNC_TSHAPE` | `GYRO_TSHAPE` |
| turn | `PIPELINE_TURN_MODE` | `TurnMode` | `COUNTER_ROT` | `PIVOT_LEFT` |
| direction | `PIPELINE_DIRECTION_MODE` | `DirectionMode` | `DEFAULT` | `FORCE_REVERSE` |

`STOP_AT_LINE` is **not** a legal `PIPELINE_BLIND_MODE` (rejected at `validator.py:2159`) and
`RAW_FLAT` would suppress the T-shape profile the compiler payload asserts on — `GYRO_TSHAPE` is the
one non-default blind value that is both authorable and behaviorally observable.

## Contracts & Interfaces

Single source of truth for every seam crossing a task boundary.

### Signatures

- `compiler.final_heading_after(path: List[str], start_heading_deg: float, graph: nx.DiGraph, field_data=None) -> float`
  — owner: **T2**; consumers: **T4**, plus the 10 existing test call sites
  (`test_compiler.py:144,150,151,189,217,552`, `test_turn_gate_parity.py:278,311,344,369`), which
  must keep working **unmodified** because the new param is keyword-optional.
- `validator._edge_attrs(a: str, b: str) -> tuple[str, float, StopAtLineMode | None, Mapping] | None`
  — owner: **T1**, internal to `_validate_pivot_turns`. The 4th element is the **live NetworkX edge
  data mapping** `graph[a][b]`, passed straight to `resolve_blind_mode(edge, cfg)` exactly as
  `compiler.py:436-444` does. Read-only: no caller may mutate it. `_edge_attrs` is *called* at
  `validator.py:793` and `:800`, but the 3-name destructurings that must widen to 4 are at
  `:807-:808`. Its one-line docstring at `:624` must gain the 4th element.
- `validator._regime(edge_type, dest_node_type, stop_override, auto_rule_active, edge_data) -> str | None`
  — owner: **T1**. Both call sites are `validator.py:854` and `:855`.
- `validator._pipeline_cfg() -> PipelineConfig` — owner: **T1**, new closure over `field_data`
  inside `_validate_pivot_turns`. Returns `PipelineConfig.from_field_data(field_data)`, or the
  `PIPELINE_CONFIG` singleton if that raises `(KeyError, ValueError, TypeError)` **or** if the
  resolved `cfg.blind is BlindMode.STOP_AT_LINE` (see Decisions). Resolved **once**
  and stored in a closure variable next to `pipeline_direction` (`validator.py:720`), not per-edge.
- `tests/test_pipeline_field_sourcing.py::_build_field(**overrides)` — owner: **T4**; consumers:
  none (T5's lint tests do not call it). Returns a `SimpleNamespace` field-data proxy that passes
  the real `validate()` + `build_graph()` pipeline, with the four `PIPELINE_*` keys settable per the
  Data Model table. **Pivot geometry constraint:** the fixture sets
  `PIPELINE_DIRECTION_MODE: "FORCE_REVERSE"` *and* must contain a pivot triple, and those interact —
  a REAR-resolved pivot approach budgets the smaller REAR sensor-to-axle offset and will reject on
  naive geometry (see the FORCE_FORWARD fold test at `test_field_data.py:2352-2373` for the shape of
  that failure). Either size the pivot delta to fit the REAR budget, or put a per-edge
  `direction: "front"` pin on the pivot legs. Do not weaken the fixture by dropping the pivot.

### File ownership

| File | Owner task | Co-writer / consumer tasks |
|------|-----------|----------------------------|
| `tools/path_planner/validator.py` | T1 | — |
| `tools/path_planner/compiler.py` | T2 | — |
| `tools/path_planner/tests/test_compiler.py` | T2 | — |
| `tools/path_planner/build_routes.py` | T3 | — |
| `tools/path_planner/tests/test_pipeline_field_sourcing.py` | T4 | T5 **appends** to it — co-writer, serialized by `Blocked By: T4` |

### Naming

Exact strings that must match across tasks:

- New test file: `tools/path_planner/tests/test_pipeline_field_sourcing.py` (not `_parity`, not
  `_audit` — `test_sim_pipeline_parity.py` already exists and is a different file).
- Lint allowlist entries are `(repo-relative path, enclosing qualified function name)` pairs. The
  **exact** post-fix legitimate set, which T5 must encode verbatim:

  | Path | Enclosing function | Why legitimate |
  |---|---|---|
  | `tools/path_planner/pipeline_config.py` | *(whole file)* | defines the singleton and its own documented absent-key fallback |
  | `tools/path_planner/sim/runtime.py` | `resolve_speed` | `cfg: PipelineConfig = PIPELINE_CONFIG` default arg |
  | `tools/path_planner/sim/runtime.py` | `resolve_blind_mode` | default arg |
  | `tools/path_planner/sim/runtime.py` | `resolve_line_follow_mode` | default arg |
  | `tools/path_planner/sim/runtime.py` | `_pass_through_heading_eligible` | `None` → singleton fallback; no production caller passes `None` |
  | `tools/path_planner/sim/runtime.py` | `PathTrackerSim.resolve_speed` | default arg |
  | `tools/path_planner/sim/runtime.py` | `PathTrackerSim.resolve_blind_mode` | default arg |
  | `tools/path_planner/sim/runtime.py` | `PathTrackerSim.resolve_line_follow_mode` | default arg |
  | `tools/path_planner/validator.py` | `_validate_pivot_turns.<locals>._pipeline_direction_default` | per-field degrade fallback |
  | `tools/path_planner/validator.py` | `_validate_pivot_turns.<locals>._pipeline_cfg` | whole-config degrade fallback (new, T1) |

  Note the six `sim/runtime.py` default-arg entries are attributed to the function being defined
  even though the default expression evaluates at module scope — that is how a function-name-stack
  AST visitor sees them, and T5 must document it.

  **The allowlist is only load-bearing if the lint flags bare `Name` loads too.** A default arg
  `cfg: PipelineConfig = PIPELINE_CONFIG` and `_pipeline_cfg`'s `return PIPELINE_CONFIG` are plain
  `ast.Name` nodes, not `ast.Attribute` — an attribute-only pattern would never flag them, leaving
  7 of the 10 entries dead and letting an alias reintroduce the bug (`c = PIPELINE_CONFIG;
  c.blind`). T5 therefore flags **both** forms; see the T5 task row for the exact node predicate.
- Lint failure message must contain the literal substring `PipelineConfig.from_field_data` so the
  fix is discoverable from the assertion alone.
- Test-directory scan root is `tools/path_planner/`, **excluding** `tools/path_planner/tests/`
  (test files legitimately monkeypatch and compare against the singleton — e.g.
  `test_field_data.py:1983`, `test_sim_pipeline_parity.py:74`).

## Error Handling

- **Malformed `PIPELINE_*` name reaches `_validate_pivot_turns`:** `_pipeline_cfg()` swallows the
  `KeyError`/`ValueError`/`TypeError` and returns the singleton; the pivot pass completes on
  default assumptions and `_validate_pipeline_config` (step 13) reports the precise
  `FieldDataError`. Never a raw exception escaping the pivot pass.
- **`field_data` is `None` in `final_heading_after`:** `from_field_data(None)` returns the
  singleton-equivalent config (all four `getattr` lookups miss). No branch needed, no crash.
- **Edge has no `mode` key:** `resolve_blind_mode` → `_resolve_override` returns `cfg.blind`. A
  `mode` equal to `BlindMode.DEFAULT` resolves identically. Both already handled by the shared
  resolver; T1 adds no new branch.
- **`build_routes.py` diagnostic when `validate()` raises:** the log never prints, because it now
  sits after `validate()`. This is intended — the diagnostic must not claim a config the field
  cannot legally express.
- **Lint encounters an unparseable `.py` file:** fail the test loudly with the filename rather than
  skipping it. A silently-skipped file is a hole in the guard.

## Testing Strategy

**Levels:** Unit (resolver + validator behavior), Integration (cross-surface agreement), Lint (static
scan).

| ID | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | Field sets `PIPELINE_DIRECTION_MODE: "FORCE_REVERSE"`; call `final_heading_after(..., field_data=field)` and `compile_path(..., field_data=field)` on the same path | Integration | Both report the same final body heading (the REAR-resolved one, edge heading + 180°). Fails before T2. |
| TC2 | Same path, `field_data=None` on both | Integration | Both report the FRONT-resolved heading — identical to today's output (no regression). |
| TC3 | Field sets all four `PIPELINE_*` non-default per the Data Model table; run `validate()` → `build_graph()` → `compile_path()` → `final_heading_after()` → `emit_routes()` on **one** field, in one test | Integration | All four surfaces agree simultaneously (this is the issue's clause-(d) ask, so it must be one test, not three): emitted `routes.cpp`'s `const PipelineConfig PIPELINE_CONFIG = {` block carries `FAST`/`GYRO_TSHAPE`/`PIVOT_LEFT`/`FORCE_REVERSE`; the compiled primitives' blind payloads carry `GYRO_TSHAPE`; `final_heading_after(..., field_data=field)` equals the final heading `compile_path` arrives at; and `validate()` accepts the field. |
| TC4 | Field omits all four keys; same pipeline | Integration | Every surface resolves to the singleton defaults; emitted block is byte-identical to the current baseline. |
| TC5 | BLIND pivot leg with per-edge `mode: "GYRO_TSHAPE"`, field `PIPELINE_BLIND_MODE: "RAW_FLAT"`; run `validate()` | Unit | Passes. `_regime` returns `"distance"` for both (neither is a seek trigger) — proves threading `cfg` changed no verdict, which is the ADR-064 invariant. |
| TC6 | Pivot leg with `stop_at_line: "on"`, field `PIPELINE_BLIND_MODE: "GYRO_TSHAPE"` | Unit | `_regime` returns `"sensor"` — the seek still comes from the `stop_at_line` axis (arg2), unaffected by the blind axis. |
| TC7 | Field sets `PIPELINE_BLIND_MODE: "NONSENSE"` on a field that also has pivot turns | Unit | Raises `FieldDataError` mentioning `PIPELINE_BLIND_MODE`, **not** `KeyError`. Guards the ADR-029 diagnostic contract (AC2). |
| TC8 | Field sets `PIPELINE_BLIND_MODE: "STOP_AT_LINE"` | Unit | Still raises the ADR-064 `FieldDataError` ("author stop_at_line: on"). Proves the fix did not re-open the retired path. |
| TC9 | Lint scan over the post-fix `tools/path_planner/` tree | Lint | Zero findings. |
| TC10 | Lint scan over a synthetic source string containing `x = PIPELINE_CONFIG.blind` inside a non-allowlisted function | Lint | Exactly one finding; message contains the line number and `PipelineConfig.from_field_data`. Negative-path content assertion, not just "it failed". |
| TC11 | Lint scan over a synthetic source containing `PIPELINE_CONFIG.blind` only inside a docstring and a comment | Lint | Zero findings — proves the AST walk beats the regex approach. |
| TC12 | `_pass_through_heading_eligible` called with `pipeline_default=None` | Unit | Still falls back to `PIPELINE_CONFIG.direction` (existing behavior preserved; the allowlist entry is load-bearing, not vestigial). |
| TC13 | Run `build_routes.py` and capture stdout | Integration | The `pipeline_config:` line contains all four category names including `direction=`, appears **after** the `validating field_data` line, and reports the field-sourced values. Covers AC4's substance, which nothing else asserts. |

**Test data:** `_build_field(**overrides)` in the new file — a `SimpleNamespace` field proxy modeled
on `test_header_writer.py:791` `_small_pipeline_field()` and `test_sim_pipeline_parity.py:79`
`_build_fixture()`. It must carry every attribute the strict emitter reads (see the
`gotcha-strict-emitter-field-fixtures` warning in Notes) and must include at least one BLIND edge and
one pivot triple so the validator and compiler paths are both live.

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/` (full suite — see Notes).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Thread field-sourced `cfg` into `validator._regime` | — | med | `tools/path_planner/validator.py` | Add a `_pipeline_cfg()` closure inside `_validate_pivot_turns` per the Contracts signature — it degrades to the singleton both on `(KeyError, ValueError, TypeError)` **and** when the resolved `cfg.blind is BlindMode.STOP_AT_LINE` (a valid enum member that `from_field_data` accepts, so the exception guard alone would not catch it; step 13 is the real reporter). Resolve it **once** into a closure var beside `pipeline_direction` (`:720`). Extend `_edge_attrs` (`:623-632`) to return the live `graph[a][b]` mapping as a 4th tuple element, widen its one-line docstring at `:624`, and update the two 3-name destructurings at `:807-:808` (the `_edge_attrs` calls themselves are at `:793`/`:800`). Add an `edge_data` param to `_regime` (`:634`) and update both call sites (`:854`, `:855`). In `_regime`'s `EDGE_BLIND` branch, replace the two `PIPELINE_CONFIG.blind` args (`:688`, `:689`) with `resolve_blind_mode(edge_data, cfg)` and `cfg.blind` respectively — import `resolve_blind_mode` alongside the existing deferred `sim.runtime` imports at `:579-585`. Correct the now-false `_regime` docstring claim at `:662-665` that per-edge mode "no longer feeds this classification". Comment the deliberate `_pipeline_cfg` vs `_pipeline_direction_default` asymmetry (see Decisions). Satisfies AC1, AC2. |
| T2 | Field-source `compiler.final_heading_after` | — | med | `tools/path_planner/compiler.py`, `tools/path_planner/tests/test_compiler.py` | Add the optional `field_data=None` param per the Contracts signature. Replace the deferred `PIPELINE_CONFIG` import (`:167`) with `PipelineConfig`, resolve `cfg = PipelineConfig.from_field_data(field_data)`, and use `pipeline_default=cfg.direction` at `:186`. Rewrite the docstring at `:137-143`: the "can never disagree with `compile_path`" guarantee now holds **only when both are given the same `field_data`** — state that explicitly. Thread `field_data` through the `_walk_expected_headings` mirror helper at `test_compiler.py:445` (which reads `_runtime._pipeline_config.PIPELINE_CONFIG.direction` and would otherwise silently mirror the old bug). Do **not** modify the 10 existing call sites — the kwarg default keeps them valid; if any breaks, the default is wrong. Satisfies AC3. |
| T3 | Field-source the `build_routes` diagnostic | — | low | `tools/path_planner/build_routes.py` | Move the `log(f"pipeline_config: ...")` call (`:291-295`) to **after** `validate(field_data)` (`:298`). Source it from `PipelineConfig.from_field_data(field_data)` and print all four categories including `direction`, which is currently missing. Remove the `# noqa: F401` `PIPELINE_CONFIG` import at `:69` (verify no other read remains in the file first) and import `PipelineConfig` instead. Satisfies AC4. |
| T4 | Cross-surface agreement test + fixture | T1, T2 | med | `tools/path_planner/tests/test_pipeline_field_sourcing.py` | Create the file. Implement `_build_field(**overrides)` per the Contracts signature and Data Model table. Implement TC1–TC8 and TC12. The fixture must survive the real `validate()` — model it on `test_header_writer.py:791` `_small_pipeline_field()` plus `test_sim_pipeline_parity.py:79` `_build_fixture()`, and include a BLIND edge and a pivot triple. TC3 slices the `const PipelineConfig PIPELINE_CONFIG = {` block out of the emitted cpp text exactly as `test_header_writer.py:845` does (also `:870`, `:891`). Satisfies AC5. |
| T5 | AST lint scan for singleton reads | T3, T4 | med | `tools/path_planner/tests/test_pipeline_field_sourcing.py` | Append to the file T4 created. Walk every `.py` under `tools/path_planner/` **excluding** `tests/` with `ast.parse` + a `NodeVisitor` maintaining a function-name stack. **Node predicate — flag both forms:** (a) any `ast.Name` with `id == "PIPELINE_CONFIG"` in a **Load** context (Store is the singleton's own definition), which catches default args, `return PIPELINE_CONFIG`, and aliasing; and (b) any `ast.Attribute` with `attr == "PIPELINE_CONFIG"` (the dotted `_pipeline_config.PIPELINE_CONFIG.direction` form at `sim/runtime.py:2041`). `from … import PIPELINE_CONFIG` produces no `Name` node, so imports are not flagged — intentional, since an import with no read is dead. Suppress a finding when its `(path, qualified function name)` is in the allowlist table under Contracts → Naming; encode that table verbatim as a module-level constant. Unparseable file → fail loudly naming the file, never skip. If TC9 reddens on a site the allowlist missed, **add it with a written justification comment** — do not loosen the predicate. Implement TC9, TC10, TC11. Satisfies AC6. |
| T6 | Full-suite verification + build smoke | T3, T5 | low | *(no file edits)* | Run `uv run --extra dev pytest tools/path_planner/tests/` (FULL — not per-file). Run `uv run python tools/path_planner/build_routes.py` once, **capture its stdout**, and confirm: it completes; the `pipeline_config:` line names all four categories including `direction=` and sits after the `validating field_data` line (TC13); and `routes.h`/`routes.cpp` are byte-unchanged (`git diff --exit-code WRO2026/src/nav/routes.h WRO2026/src/nav/routes.cpp`). Report any failure with output rather than patching around it. Satisfies AC4, AC7. |

## Notes for Implementer

**Worktree mode (Diego asked for this explicitly).** Create the worktree with raw git —
`git worktree add -b fix/pipeline-config-field-sourcing <path> main` — and drive all git from the
orchestrator via `git -C <worktree>`. Sub-agents in a sibling worktree get **Write AND Bash denied**
by the sandbox classifier, foreground *and* background. Run the author/apply model: code-writers
`Read` and **return** exact `old_string → new_string` edits (or full new-file content); the
orchestrator applies every write and owns all git. Returned content is HTML-escaped — decode
`&lt;` / `&gt;` / `&amp;` before applying. Prefer `Bash ls` / `Grep` over `Glob` for enumerating the
worktree (Glob does not resolve sibling-worktree paths).

**The worktree has no `.venv`** — it is gitignored and lives only in the main checkout. Run
`uv sync --project <worktree>` before the first pytest, or every Python task fails with
`ModuleNotFoundError: networkx`.

**Always `--extra dev`.** A bare `uv run pytest` resolves to the system Python and dies on
`ModuleNotFoundError: libcst`. The command is `uv run --extra dev pytest tools/path_planner/tests/`.

**Run the FULL pytest suite, never per-file.** T4's new `SimpleNamespace` fixture is exactly the
shape that trips `gotcha-strict-emitter-field-fixtures`: `header_writer.py` reads `field_data`
attributes strictly (bare `field_data.X`, no `getattr` default), so a fixture missing an attribute
raises `AttributeError` in a *different* test file than the one you are editing. Expect 2–3 rounds of
whack-a-mole if you iterate file-by-file.

**`build_routes.py` is the hidden consumer.** The `pipeline-config-rename-build-routes-diagnostic`
gotcha is precisely about this file: no test covers its `log()`, and the routes-freshness gate misses
it because the print does not change `routes.h`/`.cpp`. T6's manual run is the only thing that proves
T3 did not break the CLI.

**Host C++ ctest is not required** — the validator change adds no new rejection path and
`routes.cpp` is unchanged. But if any task ends up introducing a rejection the validator did not
have before, `gotcha-field-contract-change-breaks-cpp-and-py-fixtures` applies and the full
`ctest` becomes mandatory (the macOS box has the whole toolchain; use the default Unix Makefiles
generator, `ninja` is not installed, and build to a dir **outside** the worktree).

**Do not "fix" the `sim/runtime.py` default args.** They look like the same smell but are not: every
production caller threads a real `cfg`, and the allowlist documents each one. Removing them was
considered and rejected (see Scope → Out of scope).

**Do not touch `_pipeline_direction_default`.** It is already correct and a test pins its
degrade-to-`DEFAULT` behavior. The new `_pipeline_cfg` sits beside it, not on top of it.

**PR conventions** (`CONTRIBUTING.md`): branch `fix/pipeline-config-field-sourcing`, Conventional
Commit subject (`fix(planner): ...`), rich-Markdown PR body covering *What this PR does* / *How it
affects the robot* (say plainly: **no runtime effect** — offline-tooling correctness only, and the
`final_heading_after` divergence is latent with no production caller) / *How to manually test*.
Include the literal line `Doxygen reviewed for affected APIs before implementation.` and
`Closes #148`. No AI-attribution trailers.

**Rollback:** every change is additive or local to one function. Reverting the branch restores the
prior behavior exactly; nothing is generated, migrated, or persisted.
