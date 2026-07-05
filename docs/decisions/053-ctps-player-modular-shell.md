# ADR-053: CTPS player is modular concatenated assets + extension registries + `PLAYER_META`; CSS custom properties via `setProperty` are not an inline-style violation

**Status:** Implemented
**Date decided:** 2026-07 (issue #125)

## Rule

The CTPS player is built from modular `tools/path_planner/ctps/assets/*.js` + `*.css`
files that `player_html.render()` concatenates, in a fixed load order, and inlines into
the single self-contained offline HTML it emits — no bundler, no ES modules, no external
`src=`/`href=` references. The player exposes five extension **registries** — layers,
render modes, timeline lanes, panel tabs, inspector fields — plus a Python-emitted
`PLAYER_META` inline constant (source badge, filename, lane/tab manifests, default render
mode), so a future feature registers a descriptor instead of rewriting the runtime.
Dynamic, per-frame styling (playhead position, fill fraction, …) is done exclusively via
**CSS custom properties written through `element.style.setProperty('--foo', …)`**, never a
static inline `style="..."` attribute or an `el.style.<prop> =` assignment; this is not a
violation of the project's no-inline-styles convention, which targets hand-authored static
markup, not per-frame animated state. An emit-time self-containment lint in `render()`
rejects `src=`, `href=`, `http://`, `https://`, `data:image`, or `</script>` in any
concatenated asset or the HTML template itself.

## Why

The player was outgrowing a single `player.js` (~1.5-2.5k lines mixing seek, rendering,
transport, and narration); every future feature would have landed as a merge conflict in
one file. Splitting into one file per concern (core engine, store, renderer, transport,
timeline, panels, selection, chrome, boot, plus token/layout/component stylesheets) keeps
each concern independently reviewable while `render()` still ships one `.html` a judge can
open on `file://` with no toolchain. The registries exist because three feature issues are
already queued against this player — **#57** (object authoring/cargo, a HOLDING lane +
panel shell), **#55** (pivot wheel-arc rendering), and **#23** (LRS/telemetry as a second
data source) — and none should require touching the pinned seek core or the boot sequence
to land. The `renderModes` registry ships with exactly one entry (`box`) today; that is not
speculative generality, it is the seam #55 needs to add `wheel-arc` as a pure registration
with no change to `renderer.js`'s resize/transform/hit-test code. The CSS-custom-property
carve-out exists because the playhead, seek fill, and trail overlay need a per-frame
numeric value only JS knows; a hand-written inline `style=` attribute is exactly the
anti-pattern the no-inline-styles rule bans, but a `setProperty` call into a
stylesheet-declared custom property is not — the stylesheet still owns every visual rule,
JS only supplies the one number the stylesheet cannot know ahead of time.

## How to apply

- `tools/path_planner/ctps/player_html.py` — `_ASSET_CSS`/`_ASSET_JS` tuples list files in
  load order (`player.js` first, `boot.js` last); the CSS assets concatenate into a single
  `<style>`, while each JS asset is emitted in its own `<script>` element so one throwing
  module can't abort the others. `_assert_self_contained`
  runs before write and lints assets + the HTML template string only, never the embedded
  `TRACK`/`PLAYER_META`/`MAP_IMAGE` JSON values (which carry arbitrary user text).
- `tools/path_planner/ctps/assets/*.js` — `window.CTPS.{layers,renderModes,timelineLanes,
  panelTabs,inspectorFields}` each expose `register(descriptor)` (records a duplicate id
  into `CTPS._errors`, never throws) + iteration; a module's IIFE pushes its `init(ctx)` onto
  `CTPS._inits` and/or calls `register`. `assets/player.js` keeps the pinned seek core
  (`findKeyframeIndex`/`poseAt`/`findEventIndex`/`wrapDeltaDeg`) byte-stable for TC12.
- `assets/tokens.css` is the single source of every `--ctps-*` custom property, static
  tokens and the dynamic `--ctps-frac` default alike; the renderer resolves all canvas
  colors from it via `getComputedStyle`, never a hard-coded hex in JS.
- A new registry consumer (#57/#55/#23) registers a descriptor from its own asset file and
  adds that file to the emitter's asset tuple — it does not edit `player.js`, `boot.js`, or
  another feature's file.
- Per-frame visual state is written with `el.style.setProperty('--ctps-<name>', …)` against
  a custom property `tokens.css` (or the consuming stylesheet) already declares; a static
  inline `style="..."` attribute or an `el.style.<camelProp> =` assignment stays banned and
  is caught by the TC-NOINLINE emitter test.

## Trigger to revisit

- A real bundler or dev-server becomes necessary (asset count or shared-state needs outgrow
  string concatenation) — revisit the "no bundler, no ES modules" half of the rule.
- A live-data source (e.g. a websocket-fed LRS/telemetry stream, #23) needs to update the
  player after the single-file emit — revisit the "one self-contained offline HTML" premise,
  since a live source is not a `file://`-openable static artifact.

## See also

- [ADR-033 — CTPS ingests a derived mission trace recorded from the real C++](./033-ctps-derived-mission-trace.md) — the single-file offline `.html` player whose internals this ADR restructures.
- [ADR-051 — CTPS mock-builder surface conformance guards](./051-ctps-mock-conformance-guard.md) — the sibling CTPS drift guard (mock builders vs. real builders) alongside this one (registries vs. registered features).
- Issues #125 (this rework), #57 (object authoring/cargo — HOLDING seam), #55 (pivot wheel-arc — renderModes seam), #23 (LRS/telemetry — source/registry seam).
