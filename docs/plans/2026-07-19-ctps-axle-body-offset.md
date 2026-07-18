# CTPS Robot Axle→Body Offset Plan

**Goal:** Fix CTPS player fidelity so the robot body (chassis square + heading arrow), the vision cone, the mosaic visibility HUD, and the manipulator zone overlays are drawn at the true body-centre — offset from the rotation axle by the real axle→centre distance — while the axle stays anchored on the graph node.

**Approach:** Add an optional `axle_offset = {forward_cm, lateral_cm}` block to the layout `robot` section (`ctps/layout.py`). Compute a shared draw-time "body pose" = `axle_pose` shifted by `localToWorld(pose, forward_cm, lateral_cm)` (a new `CTPS_OBJ.axleToBodyPose` helper that reuses the existing single robot-local→world transform), and feed that body pose to every robot-local *draw* site: the body marker (`renderer.js`), the cone + visibility HUD (`vision.js`), and the zone overlays (`cargo.js`). The axle pose is untouched for the trajectory, trail, node/hit anchoring, and the cargo event *resolution*, so no cargo-sim/golden change. Resolves issue #165.

## Scope

- **In scope:**
  - New optional `robot.axle_offset` layout block (`forward_cm`, `lateral_cm`, robot-local) with validation and byte-identical-when-absent `to_dict` in `ctps/layout.py`.
  - Shared `CTPS_OBJ.axleToBodyPose(pose, robotCfg)` helper in `objects_draw.js`.
  - Apply the body pose to the robot body marker (`renderer.js`), the vision cone + `k/12` mosaic visibility HUD (`vision.js`), and the grab/box/drop zone overlays incl. deployed zones (`cargo.js`) — draw sites only.
  - CTPS layout editor round-trip preservation of `axle_offset` (`editor_io.js` ingest + serialize) — prevents silent data loss on load→save.
  - Tests (layout validation/round-trip, player embed + no-layout regression, editor round-trip) and a mandatory manual eyeball verification.
- **Out of scope:**
  - Any firmware / runtime / planner / sim (`sim/runtime.py`) behavior — this is pure visualization fidelity (issue: "no runtime/firmware effect").
  - The cargo grab/dump **event resolution** (`cargo_resolve` / `_doGrab` / `_doDump` / `_doBoxClose` / `_doBoxOpen`) — stays on the axle pose (user decision), so cargo scene goldens/fixtures are unchanged.
  - Re-introducing any per-junction node offset — the axle-on-node invariant stays (issue note; [[gotcha-sim-chassis-axle-on-node]]).
  - The layout **editor's robot preview backdrop** (`editor_core.js`, drawn at identity pose) — it keeps drawing centred; the editor only *preserves* the field on round-trip, it gains no offset-editing control or offset preview.
  - Sourcing the offset from `ctps/config.py`/`TRACK.field` (an always-on alternative that was considered and rejected — see Decisions).

## Decisions

- **Offset lives in the layout `robot` block** (not `ctps/config.py`/`TRACK.field`) — user choice, matches the issue text and co-locates with the other robot-local geometry (vision cone origin, zones). **Consequence, accepted and loud:** the offset applies ONLY when a layout with a non-zero `axle_offset` is loaded. Every render with **no layout** — the common `--trace` / `--fixture` / any navigation-only mission path — keeps the mispositioned centred body, i.e. issue #165's defect stays uncorrected on the most common render path. The axle→centre distance is a fixed robot constant, so an always-on `config.py`/`TRACK.field` source would have fixed every render; that alternative was explicitly weighed and rejected in favor of the layout source. Default absent ≡ `{0,0}` ≡ today's render.
- **Adding a non-zero `axle_offset` redefines the robot-local origin as the body centre — existing robot-local geometry must be re-authored.** Today robot-centre == axle == pose, so any cone origin (`robot.vision.origin_x_cm/y_cm`), zone centre, or drop point authored against the current render is de-facto measured **from the axle**. Once `axle_offset` shifts the robot-local origin to the body centre, those same coordinates are re-interpreted relative to the body centre — a layout that had the axle→camera distance baked into its cone origin would **double-count** it. Authoring a non-zero offset therefore requires re-checking/re-authoring the cone origin, zones, and drop points against the body centre. (A zero/absent offset is unaffected — the origin stays the axle.)
- **Body pose moves the body marker + cone + mosaic-vis HUD + zone overlays; the cargo event resolution stays on the axle** — user choice (visual consistency without changing which objects resolve into zones, so cargo goldens stay unchanged). **Accepted tradeoff, stated honestly:** the *drawn* grab/box zones ride the body while the *resolved* zones (`cargo_resolve`) use the axle, so the two disagree by the full offset vector. At a zone EDGE this is observable — the player can draw a tile inside the grab-zone overlay whose grab event does not resolve, or resolve a grab for a tile just outside the drawn zone. This is a known limitation of keeping resolution frozen; it is not "they effectively agree." Moving resolution too (Q2 option C) was rejected to avoid rewriting cargo goldens.
- **Reuse `localToWorld` for the offset math** — `axle→body-centre` in world is exactly `localToWorld(pose, forward_cm, lateral_cm)`, so `axleToBodyPose` is single-sourced against the existing transform (no drift across the 4 draw sites).
- **`player_html.py` needs no code change.** `_objects_script` embeds the raw layout dict byte-for-byte (`json.dumps(objects)`) after `layout.validate`. Correction to an earlier assumption: `_validate_robot` does **not** reject unknown keys today, so an `axle_offset` block already validates-and-embeds with zero Python change. T1's real value is (a) **strict** validation that rejects a *malformed* offset and (b) the typed `AxleOffset` dataclass on both the `validate` and the `load`/`_build_layout` construction paths (so `load(dict).to_dict()` round-trips it — see T1). This satisfies the issue's "thread through `player_html.py`" without an edit.
- **No automated test of the JS helper math or any JS behavior** — CTPS JS assets are text/source-asserted only (no JS runner; `test_ctps_layout_editor.py`'s own harness exercises assets "as text — no browser, no compiler"). Every JS behavioral claim (helper math, cone/zone/HUD reposition, editor round-trip) rests on source-text tamper-pins + review + the mandatory manual eyeball gate (T10, both player and editor legs).
- **No UI/UX design gate** — this is a canvas-geometry correction to an existing player, not a new page/screen/component/flow.

## Acceptance Criteria

- [ ] AC1 — A layout with `robot.axle_offset = {forward_cm: F, lateral_cm: L}` draws the chassis square + heading arrow centred at `axle + Rot(h)·(F, L)`; the trajectory line, traveled trail, and node dots stay on the axle (on the node).
- [ ] AC2 — The vision cone apex and the `k/12` mosaic visibility HUD are computed from the offset body pose (cone/camera stays attached to the body).
- [ ] AC3 — The grab / box / drop-point zone overlays (incl. deployed zones) draw at the offset body pose.
- [ ] AC4 — The cargo grab/dump event resolution is unchanged (uses the axle pose); the cargo scene timeline / fixtures / goldens are byte-identical.
- [ ] AC5 — A layout with no `axle_offset` (or `{0,0}`) renders the body/cone/zones exactly as today (the `axleToBodyPose` helper returns the pose unchanged), and its emitted `const OBJECTS` block round-trips identically (no `axle_offset` key). A render with no layout is unchanged (no `const OBJECTS`).
- [ ] AC6 — `ctps/layout.py` validates `axle_offset`: absent is OK; when present it requires finite numeric `forward_cm` and `lateral_cm` and raises `LayoutError` naming the key path on anything else. `RobotConfig.to_dict()` emits `axle_offset` only when configured (absent ⇒ key omitted). `load(dict_with_offset).to_dict()` round-trips the offset byte-for-byte (both the `validate` and `_build_layout` paths carry it).
- [ ] AC7 — The CTPS layout editor round-trips `axle_offset`: its `_serialize` source emits an `axle_offset` block when the loaded state carries one, and its ingest (`_parseAxleOffset`) reads `robot.axle_offset` into editor state — so loading a layout carrying an offset and re-saving preserves it (no silent drop). Verified by source-text assertions (T9) + the T10 editor eyeball leg.
- [ ] AC8 — `PLAYER_META` is untouched and no `const OBJECTS` is emitted for a no-layout render (`test_ctps_player_html.py` TC9/TC-META literals stay green). (The emitted PAGE bytes DO change because the four edited JS assets are inlined — there are no full-page byte goldens; only PLAYER_META + structure are pinned.)
- [ ] AC9 — Manual eyeball: a rendered player with a non-zero offset visibly shows the body / cone / zones shifted off the node while the axle stays on the node.

## UI/UX Specification

n/a — no new frontend surface. This is a geometry correction to the existing CTPS player canvas; no new layout, colors, components, or flows.

## Data Model

New optional block on the layout `robot` section (`ctps/layout.py`). Robot-local frame: origin = robot centre, `+x` = heading-forward, `+y` = robot-left (same convention as `RectZone` / `VisionCone`).

```python
@dataclass
class AxleOffset:
    """Axle→body-centre offset (robot-local). The body is drawn shifted from the
    axle by this vector; the axle stays anchored on the node."""
    forward_cm: float   # robot-local +x: + moves the body ahead of the axle
    lateral_cm: float   # robot-local +y: + moves the body to the robot's left

    def to_dict(self) -> dict[str, float]:
        return {"forward_cm": self.forward_cm, "lateral_cm": self.lateral_cm}

@dataclass
class RobotConfig:
    grab_zone: RectZone
    box_zone: RectZone
    drop_points: list[DropPoint]
    vision: Optional[VisionCone] = None
    axle_offset: Optional[AxleOffset] = None      # NEW — absent ⇒ {0,0}
    # to_dict(): emit "axle_offset" ONLY when not None (mirrors the "vision" rule)
```

**`layout.py` has TWO serialization paths that must BOTH learn `axle_offset`** (the same dual-path trap as [[gotcha-fieldmodel-dual-serialization]]):
1. **`validate(data)`** — the `_validate_robot` → `_validate_axle_offset` check path.
2. **`load(data)` / `_build_layout`** — the dict → dataclass **construction** path (the `_build_vision` analogue). This must gain `_build_axle_offset` and wire it into the `RobotConfig` build. If only `validate` learns the field, `load(dict_with_offset).to_dict()` silently **drops** `axle_offset` — reproducing on the Python side the exact data-loss T6 fixes on the editor side, and breaking `Layout.to_dict`'s documented byte round-trip.

Wire JSON shape (embedded in `const OBJECTS`):

```json
"robot": {
  "grab_zone": { "x_cm": ..., "y_cm": ..., "w_cm": ..., "h_cm": ... },
  "box_zone":  { ... },
  "drop_points": [ ... ],
  "vision": { ... },
  "axle_offset": { "forward_cm": 4.0, "lateral_cm": 0.0 }
}
```

## Contracts & Interfaces

### Layout schema seam — `robot.axle_offset`

`robot.axle_offset = { "forward_cm": number, "lateral_cm": number }` — robot-local (`+forward` = heading-forward, `+lateral` = robot-left). Optional; absent ≡ `{0,0}`. Both keys required together, finite numbers.

- **Owner:** `ctps/layout.py` (T1) — dataclass + `_validate_axle_offset` (validate path) + `_build_axle_offset` (load/`_build_layout` construction path) + `to_dict` emit-when-present.
- **Consumers:** `editor_io.js` (T6, mirror key-for-key on ingest + serialize); `test_ctps_layout.py` (T7); `test_ctps_player_html.py` (T8); the JS body-pose helper (reads `robotCfg.axle_offset.forward_cm` / `.lateral_cm`).

### JS helper seam — `CTPS_OBJ.axleToBodyPose`

```
CTPS_OBJ.axleToBodyPose(pose, robotCfg) -> pose
```

Returns a shallow COPY of `pose` with `x`/`y` shifted to the body centre and `h` unchanged, when `robotCfg.axle_offset` is present and non-zero; returns the SAME `pose` object unchanged when there is no offset (no allocation, byte-identical behavior). Body centre reuses the existing transform: `{x,y} = CTPS_OBJ.localToWorld(pose, forward_cm, lateral_cm)`. Reads `forward_cm`/`lateral_cm` defensively (missing/NaN ⇒ 0). Never throws (non-fatal-overlay contract, like `_applyLateralOffset`).

- **Owner:** `objects_draw.js` (T2).
- **Consumers:** `renderer.js` (T3), `vision.js` (T4), `cargo.js` (T5).

### Store seam (no change)

`store.robotCfg` already carries the whole `robot` block wholesale (`store.loadLayout: store.robotCfg = layoutDict.robot`), so `store.robotCfg.axle_offset` reaches all three JS consumers with no store.js edit.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `tools/path_planner/ctps/layout.py` | T1 | T6, T7, T8 |
| `tools/path_planner/ctps/assets/objects_draw.js` | T2 | T3, T4, T5 |
| `tools/path_planner/ctps/assets/renderer.js` | T3 | — |
| `tools/path_planner/ctps/assets/vision.js` | T4 | — |
| `tools/path_planner/ctps/assets/cargo.js` | T5 | — |
| `tools/path_planner/ctps/assets/editor_io.js` | T6 | T9 |
| `tools/path_planner/tests/test_ctps_layout.py` | T7 | — |
| `tools/path_planner/tests/test_ctps_player_html.py` | T8 | — |
| `tools/path_planner/tests/test_ctps_layout_editor.py` | T9 | — |

### Naming (must match byte-for-byte across Python + JS)

- Layout keys: `axle_offset`, `forward_cm`, `lateral_cm`.
- JS helper: `CTPS_OBJ.axleToBodyPose`.

## Error Handling

- **Malformed `axle_offset`** (non-dict, missing a key, non-finite/non-numeric value) → `layout.validate` raises `LayoutError` naming the offending key path (e.g. `ctps.layout: robot.axle_offset.forward_cm must be a finite number`). Player render fails loudly rather than drawing a wrong body.
- **`robotCfg` null (no layout) or `axle_offset` absent/zero** → `axleToBodyPose` returns the pose unchanged; body/cone/zones draw at the axle exactly as today.
- **`window.CTPS_OBJ` not yet defined when `renderer.js` draws** → guard the call (`window.CTPS_OBJ && CTPS_OBJ.axleToBodyPose`); on absence fall back to the raw axle pose. (Draw runs post-boot, so `CTPS_OBJ` is defined in practice; the guard covers the load-order edge and keeps `renderer.js` non-fatal.)
- **Editor loads a malformed `axle_offset`** → tolerant ingest (like `_parseVision`): warn to console and drop to no offset; never throw (matches editor's malformed-block tolerance).
- **Player UPLOAD leg (ADR-056) is unvalidated by design** → an in-browser uploaded layout is stashed raw (`store.loadLayout: store.robotCfg = layoutDict.robot`) and never runs Python `validate`, so a malformed uploaded `axle_offset` reaches the JS helper directly. The helper degrades safely (missing/NaN key ⇒ 0) rather than throwing — the same tolerance the uploaded `mosaic`/`vision` blocks already rely on. This is intentional, not a gap.
- **Intentionally divergent malformed-block semantics across surfaces** (document, don't unify): the Python emit path REJECTS (`LayoutError`, both keys required), the JS helper HALF-APPLIES (missing/NaN ⇒ 0, safe degrade), the editor DROPS-and-warns. This mirrors how `vision`/`mosaic` already behave (Python strict, JS tolerant) and is expected — the same malformed input renders differently depending on which surface loaded it.

## Testing Strategy

**Levels:** Unit (Python layout — the only behaviorally-testable surface), Source-text pins (JS — no JS runner; assert asset source, tamper-evidence between eyeballs), Manual (eyeball — the only gate that actually sees the render, both player and editor legs).

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `RobotConfig` with `axle_offset={forward_cm,lateral_cm}` → `to_dict()` | Unit | `axle_offset` present with both keys; validates clean |
| TC2 | `RobotConfig` with no `axle_offset` → `to_dict()` | Unit | `axle_offset` key omitted entirely (identical to today) |
| TC3 | `validate` on a layout with valid `robot.axle_offset` | Unit | passes |
| TC4 | `validate` with `axle_offset` missing a key / non-finite / non-dict / not-a-dict | Unit | raises `LayoutError` naming `robot.axle_offset.<key>` |
| TC5 | **`load(dict_with_offset).to_dict()` round-trip** (the dual-path gate) | Unit | offset survives dict→dataclass→dict byte-for-byte; catches a missing `_build_axle_offset` in `_build_layout` |
| TC6 | `_objects_script` with a layout carrying `axle_offset` | Unit | emitted `const OBJECTS` contains the `axle_offset` block verbatim |
| TC7 | `render(objects=None)` and `render` with an `axle_offset`-free layout | Unit | `PLAYER_META` untouched; no `const OBJECTS` for the None case; TC9/TC-META literals green (NOT a full-page byte assertion) |
| TC8 | **Text pin — helper wired at every draw site:** `renderer.js` `_drawActors`, `vision.js` cone layer + HUD, `cargo.js` zones + deployed-zones layers each reference `axleToBodyPose(` | Source-text | present in all four; tamper-evidence for AC1–AC3 |
| TC9 | **Text pin — resolution NOT moved (AC4):** `cargo_resolve` / `_doGrab` / `_doDump` / `_doBoxClose` / `_doBoxOpen` contain **no** `axleToBodyPose` reference | Source-text | absent; proves resolution stays on the axle |
| TC10 | Full cargo suite regression (`test_ctps_*` cargo/scene/track tests) | Unit | green — cargo scene timeline / fixtures unchanged (AC4) |
| TC11 | **Text pin — editor round-trip (AC7):** `editor_io.js` `_serialize` source emits `axle_offset` (gated on state) and `_parseAxleOffset` references `robot.axle_offset` with a tolerant guard | Source-text | both present; catches a dropped-on-save regression |
| TC12 | Manual eyeball — PLAYER leg: render from a layout with `axle_offset={forward_cm:8, lateral_cm:0}`, serve over `python -m http.server`, drive via `CTPS.store` | Manual | body/arrow, cone, and zone overlays sit ~8 cm ahead of the node; trail/trajectory/node dot stay on the node; scrubbing keeps body+cone+zones co-moving; hit-test selects the robot at the DRAWN body position |
| TC13 | Manual eyeball — EDITOR leg: open the layout editor on a layout carrying `axle_offset`, Save, reopen/re-serialize | Manual | the saved layout still carries `axle_offset` (no silent drop); malformed offset is tolerated (no crash) |

**Test data:** reuse the `test_ctps_layout.py` / `test_ctps_player_html.py` fixture builders; add an `axle_offset` variant. For the eyeball, render `ctps_cargo_trace.json` + a cargo layout dict augmented with `robot.axle_offset` (see [[ctps-cargo-demo-render-recipe]]).

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/test_ctps_layout.py tools/path_planner/tests/test_ctps_player_html.py tools/path_planner/tests/test_ctps_layout_editor.py` then the full `tools/path_planner/tests/` suite.

**Eyeball gate (mandatory — the text suite cannot see rendering; [[gotcha-eyeball-ctps-html-via-localhost]]):** serve `output/` over `python -m http.server` (claude-in-chrome refuses `file://`), navigate `http://127.0.0.1:<port>/page.html`. Because the eyeball tab is backgrounded/collapsed ([[gotcha-ctps-eyeball-canvas-collapsed-unfocused-tab]]), verify by (a) reading `CTPS.store.robotCfg` / `CTPS.engine.poseAt(t)` after a synchronous `seek`, and (b) calling the shipped draw fns onto a freshly `document.createElement('canvas')` of known size, then `getImageData` that offscreen buffer — do NOT measure the live stage canvas or `await` rAF.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Layout `axle_offset` schema (BOTH paths) | — | med | `tools/path_planner/ctps/layout.py` | Add `AxleOffset` dataclass (`forward_cm`, `lateral_cm`, `to_dict`). Add `axle_offset: Optional[AxleOffset] = None` to `RobotConfig`; emit in `to_dict()` ONLY when not None (mirror the `vision` pattern). **Update BOTH serialization paths:** (a) validation — add `_validate_axle_offset(robot)` called from `_validate_robot`: absent OK; when present must be a dict with finite numeric `forward_cm` and `lateral_cm`, else raise `LayoutError` naming `robot.axle_offset.<key>`; (b) construction — add `_build_axle_offset(robot)` (the `_build_vision` analogue) and wire it into `_build_layout`'s `RobotConfig(...)` build so `load(dict).to_dict()` round-trips the offset. Missing (b) reproduces the [[gotcha-fieldmodel-dual-serialization]] drop. Follows the Contracts "Layout schema seam". Satisfies AC5, AC6. |
| T2 | `axleToBodyPose` helper | — | med | `tools/path_planner/ctps/assets/objects_draw.js` | Add `CTPS_OBJ.axleToBodyPose(pose, robotCfg)` per the Contracts "JS helper seam": read `robotCfg.axle_offset.forward_cm`/`.lateral_cm` (default 0 on missing/NaN); if both 0 return `pose` unchanged; else return a shallow copy with `{x,y}` = `_localToWorld(pose, fwd, lat)` and `h` unchanged. Never throw. Satisfies AC1 (math), AC5. |
| T3 | Body marker + hit-test at body pose | T2 | low | `tools/path_planner/ctps/assets/renderer.js` | In `_drawActors`, derive the marker pose as `axleToBodyPose(engine.poseAt(t), CTPS.store.robotCfg)` FIRST, then apply the existing `_applyLateralOffset(...)` (#156) on top; pass that to the render-mode `draw`. Guard on `window.CTPS_OBJ`; fall back to raw pose on absence. **Also move the robot hit-test anchor** (`hitTest`, currently `_project(poseAt(store.t))`) to `axleToBodyPose(poseAt(store.t), CTPS.store.robotCfg)` so a click selects the robot at its DRAWN body position (an 8 cm offset × view.scale easily exceeds the 16 px `HIT_ROBOT_RADIUS`). Leave `_drawTrail`, the trajectory, and node draws on the axle pose. (The pre-existing #156 lateral-nudge hit divergence is unchanged/out of scope.) Satisfies AC1. |
| T4 | Cone + visibility HUD at body pose | T2 | med | `tools/path_planner/ctps/assets/vision.js` | At the cone layer draw and the per-frame `k/12` HUD recompute, replace the `CTPS.engine.poseAt(t)` pose with `OBJ.axleToBodyPose(poseAt(t), CTPS.store.robotCfg)` before `drawVisionCone(...)` and `visibleCells(...)`. Cone geometry (`store.visionCone`) and mosaic cells (world-frame) are unchanged. Satisfies AC2. |
| T5 | Zone overlays at body pose | T2 | low | `tools/path_planner/ctps/assets/cargo.js` | In `_drawZonesLayer` and the deployed-zones layer draw, replace the `CTPS.engine.poseAt(t)` pose with `OBJ.axleToBodyPose(poseAt(t), state.robotCfg)` before `OBJ.drawZones(...)` / `OBJ.drawDeployedZones(...)`. Do NOT touch `cargo_resolve` / `_doGrab`/`_doDump`/`_doBoxClose`/`_doBoxOpen` (resolution stays on the axle). Satisfies AC3, AC4. |
| T6 | Editor round-trip preserve | T1 | med | `tools/path_planner/ctps/assets/editor_io.js` | Mirror the editor's `vision` handling EXACTLY — vision is hoisted to a TOP-LEVEL `E.state.vision` (NOT stored inside `E.state.robot`, which `_parseRobot` returns as only `{grab_zone, box_zone, drop_points}`), then re-nested under `robotDict.vision` on serialize. Do the same: in `_ingest`, add `_parseAxleOffset(data.robot && data.robot.axle_offset)` (tolerant like `_parseVision`: malformed ⇒ warn + drop, never throw) and store to a top-level `E.state.axleOffset`; in `_serialize`, add `_axleOffsetDict(a)` and set `robotDict.axle_offset` ONLY when `E.state.axleOffset` is present — key-for-key with Python `AxleOffset.to_dict`. Do NOT try to put it inside `E.state.robot`/`_parseRobot`. Satisfies AC7. |
| T7 | Layout tests | T1 | low | `tools/path_planner/tests/test_ctps_layout.py` | Add TC1–TC5: `to_dict` with/without offset (key omitted when absent), `validate` accept, the malformed-key raise cases naming the key path, and — critically — TC5 the `load(dict_with_offset).to_dict()` round-trip that catches a missing `_build_axle_offset` in `_build_layout`. Satisfies AC6. |
| T8 | Player embed + JS source pins | T1 | low | `tools/path_planner/tests/test_ctps_player_html.py` | Add TC6 (a layout with `axle_offset` embeds the block verbatim in `const OBJECTS`); TC7 (no-layout `render` emits no `const OBJECTS` and `PLAYER_META`/TC9/TC-META literals stay green — NOT a full-page byte assertion); TC8 (source-text: the inlined `renderer.js`/`vision.js`/`cargo.js` each reference `axleToBodyPose(` at their draw sites); TC9 (source-text: `cargo_resolve`/`_doGrab`/`_doDump`/`_doBoxClose`/`_doBoxOpen` contain NO `axleToBodyPose` — proves resolution stays on the axle). Satisfies AC8; tamper-evidence for AC1–AC4. |
| T9 | Editor round-trip source pin | T6 | low | `tools/path_planner/tests/test_ctps_layout_editor.py` | Add TC11 (source-text over the inlined `editor_io.js`): `_serialize` emits an `axle_offset` block gated on `E.state.axleOffset`, and `_parseAxleOffset` references `robot.axle_offset` with a tolerant guard. (Behavioral round-trip + malformed tolerance are the T10 editor eyeball leg — no JS runner exists here.) Satisfies AC7. |
| T10 | Manual eyeball (player + editor legs) | T1, T2, T3, T4, T5, T6 | med | — (verification only) | PLAYER leg (TC12): render a cargo player from a layout with a non-zero `axle_offset`, serve over `python -m http.server`, verify per the Eyeball gate — body/arrow + cone + zones shifted off the node by the offset, trail/trajectory/node on the node, co-moving under scrub, hit-test selects at the drawn body. EDITOR leg (TC13): open the layout editor on that layout, Save, re-serialize, confirm `axle_offset` survives and a malformed offset is tolerated. Also confirm the full cargo suite is green (TC10). Satisfies AC9 + the behavioral side of AC4/AC7. |

## Notes for Implementer

- **`player_html.py` is intentionally NOT in the task list** — `_objects_script` embeds the raw dict via `json.dumps(objects)`. `_validate_robot` does NOT reject unknown keys today, so an `axle_offset` block already validates-and-embeds with zero player_html/validate change; T1 adds STRICT validation (reject malformed) + the typed dual-path round-trip. Do not add a player_html edit "to thread it through" — the thread-through is the existing embed.
- **Adding a non-zero offset requires re-authoring existing robot-local geometry** (cone origin, zones, drop points) against the body centre — today those are de-facto axle-relative, so a non-zero `axle_offset` would otherwise double-count the axle→camera distance already baked into an authored cone origin. See the Decisions migration bullet; call it out to whoever authors the first non-zero offset.
- **Reuse `localToWorld` — do not re-derive the rotation math** in `axleToBodyPose`; drift across the 4 draw sites is the exact multi-site-contract trap the project keeps hitting.
- **Compose order in `renderer.js`:** axle→body offset FIRST (physical body position), then the #156 lateral hug nudge on top. Both are draw-time pose copies; never mutate the engine's cached pose.
- **Do not change the cargo event resolution** (user decision) — only the DRAW sites move. Changing `cargo_resolve` would rewrite cargo goldens and is out of scope.
- **Editor preview backdrop (`editor_core.js`) is out of scope** — it stays centred; T6 only makes `editor_io.js` *preserve* the field on load→save so authoring doesn't silently drop it ([[gotcha-fieldmodel-dual-serialization]] is the field-editor analogue of this same round-trip trap).
- **The mosaic is world-frame** (`mosaic.py` resolves world cell centres and does not consume the cone); only the cone and the JS-only `visibleCells` HUD are robot-local, so shifting the body pose changes the `k/12` prediction (more accurate — the camera is on the body) with no Python parity to break.
- **Verification is the real gate.** The CTPS pytest suite tests JS/CSS as TEXT and never renders — a logic bug in the helper passes every green test. T10's eyeball is mandatory, using the offscreen-canvas + store-state method (the live stage canvas is collapsed in a background tab).
- **Branch note:** the current branch `feat/write-comp-task` carries unrelated uncommitted `Tasks.cpp`/`Tasks.h` changes. Execute this on a fresh branch (e.g. `feat/ctps-axle-body-offset`) so the CTPS work doesn't mix with that in-progress comp-task work.
- **Rollback:** the change is additive and gated on an optional layout field defaulting to `{0,0}`; reverting the JS/layout edits restores the centred render with no data migration.
