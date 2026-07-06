# ADR-056: CTPS object/cargo model authored on a separate offline editor page, animated by a resolve-once scene timeline

**Status:** Implemented
**Date decided:** 2026-07 (issue #57)

## Rule

Field objects (`Rect`/`Circle`, typed `tile`/`cargo`/`fixed`) and robot manipulator
zones (grab zone, rear-box zone, 4 drop points, robot-local frame) are authored in a
**new, separate offline page** — `layout_editor.html` — into one `layout.json`, never
in the CTPS player itself. The player only *consumes* a layout (emit-time embed as
`const OBJECTS`, or in-browser upload) and animates it against six manipulator verbs:
`grab` (nearest Tile whose **center** is inside the grab zone → in-hand), `sortTo(n)`
(in-hand → magazine lane `n`), `dump` (front tile of each non-empty lane → its drop
point), `boxClose`/`boxOpen` (capture/drop a Cargo at the rear-box zone, same
center-in-zone predicate, capacity 1), and `reset` (no cargo change). All object
placement over time is computed **once**, on layout-load, as an **event-indexed scene
timeline**: one snapshot per manip event (`inHand`, `magazine[4]`, `rearBox`,
`placed`), keyed by event index; scrubbing to any `t` is a pure bisect into that
array, never incremental per-tick folding. Emission is additive: `const OBJECTS` is
conditional on a layout being present (mirrors the `Track.to_dict` pivots-omission
precedent); `PLAYER_META` is unchanged; the `manip` track event gains `method`/`arg`
alongside the existing `label`. `boxOpen`/`boxClose` land on **both** the production
`Manipulator` facade and the ctps recording mock — a plain two-surface build
requirement, **not** an ADR-051 conformance claim (ADR-051's guard is the
`Drive`/`LineFollower` fluent-builder surface only).

## Why

The player's seek core (`player.js`) and `renderer.js` carry byte-frozen test
contracts (TC9's exact `PLAYER_META` literal, TC-STRUCT's DOM ids, TC-NOINLINE) that
authoring UI churn — rubber-band drawing, drag/select, undo, a properties panel —
would put at constant risk if built into the player itself. A second, standalone page
keeps that churn fully outside the frozen surface; the two pages share only *new*
files (`objects_draw.js`, `_page_emit.py`) and duplicate the ~40-line world→pixel
transform rather than extract it from `renderer.js`, which sits adjacent to the
pinned seek core and is high-risk to touch for a small, stable formula. Object
positions in this model change only *at* manip events, not continuously — so folding
state tick-by-tick (as the trail/pose interpolation does) would have to run backward
on a rewind scrub, which is exactly the incremental-state bug class the text-only
pytest harness can't see. Resolving the whole scene once, indexed by event, turns
scrubbing into the same pure-bisect idiom `findEventIndex`/the Log panel already use,
and runs the spatial geometry (nearest-tile, center-in-zone) in exactly one place.

## How to apply

- `tools/path_planner/ctps/layout.py` defines and validates `layout.json`
  (`objects[]` + robot-local `grab_zone`/`box_zone`/`drop_points[4]`); JS load is
  tolerant (skip+warn a malformed object, never throw).
- `assets/cargo.js`'s `cargo_resolve(track, objects, robotCfg)` is the single pure,
  DOM-free function producing the scene array; the object/zone layers register on
  the `trail` canvas (ADR-053 registries), never touching `renderer.js`.
- `player_html.render(objects=...)` emits `const OBJECTS` only when a layout is
  present; a layout-free render still has no `const OBJECTS`, an unchanged `TRACK`
  round-trip, and the identical `PLAYER_META` literal (TC9 stays green) — the cargo
  assets/markup ship on every page regardless, since the upload leg needs them.
- `track_builder.py` adds `method`/`arg` to the `manip` event; `trace_loader.py`
  needs no change (`manip.method` was never whitelisted).
- `WRO2026/src/manip/Manipulator.{h,cpp}` and `test/ctps_mocks/Manipulator.h` each
  gain `boxOpen()`/`boxClose()` independently; verify via `arduino-cli compile`
  (production) and the `ctps_dump` build (mock), not `test_ctps_mock_superset`.

## Trigger to revisit

- A second animated (non-manip-event) driver of object position — e.g. continuous
  physics — would break the "changes only at manip events" premise the resolve-once
  timeline depends on.
- The editor and player drift on the world→pixel transform after an unrelated change
  to one but not the other — revisit whether duplication is still cheaper than
  extracting a shared transform module.

## See also

- [ADR-053 — CTPS player modular assets + extension registries](./053-ctps-player-modular-shell.md) — the registries (`layers`, `PLAYER_META`) this feature extends without modification.
- [ADR-033 — CTPS ingests a derived mission trace](./033-ctps-derived-mission-trace.md) — the recorded-trace convention the additive `manip.method`/`arg` keys stay inside of.
- [ADR-051 — CTPS mock-builder conformance guards](./051-ctps-mock-conformance-guard.md) — the sibling drift guard this ADR explicitly does not claim; that guard covers `Drive`/`LineFollower` only.
- [ADR-032 — Manipulator API: verb-level builders](./032-manipulator-api-pattern.md) — the verb pattern `boxOpen`/`boxClose` extend, shipping as empty-mask scaffolds pending bring-up.
- `docs/plans/2026-07-06-ctps-object-cargo.md` (plan) and its UI sidecar — the originating design; issue #57.
