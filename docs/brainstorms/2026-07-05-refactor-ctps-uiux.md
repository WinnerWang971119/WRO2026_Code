# Brainstorm: Refactor CTPS player UI/UX (issue #125)

**Seed (verbatim):** We're going to design the Refactor UI/UX (issue #125). Hand
off to "Clot/Claude Design" — but first produce a SCOPING SPEC counting: how many
things, how many pages, how many abilities, how many functions to support. Then
read the two related GitHub issues and reserve them a UI/UX spot for later dev.

**Context summary:**
- CTPS today = one self-contained, offline HTML/Canvas player emitted by
  `tools/path_planner/ctps/player_html.py` + `assets/player.js`. Layout: canvas
  (field map/vector + trajectory polyline + nodes + robot arrow) | 280px side
  panel (Current command + scrolling Log) | transport row (Play/Pause, seek
  slider 0..1000, time readout).
- Hard constraints (ADR-033 + TC5): single file, everything inlined, no external
  src/href, runs on file://, CSS classes only, vanilla JS no frameworks, bounded
  binary-search seek (findKeyframeIndex/poseAt) must not regress to a full scan.
  Markers #ctps-play/#ctps-seek/#ctps-time are asserted by emitter tests.
- ADR-033 is the architecture: CTPS ingests a *derived mission trace* recorded
  from real C++ (`ctps_dump`), expanded through the parity-locked PathTrackerSim.
  Constant-velocity playback, no fault-FSM, drift flag is the only physical
  problem surfaced.
- The two related issues (ADR-033 lists BOTH as "Trigger to revisit"):
  - **#57 CTPS — Real Grabbing and putting** (P2): add a CTPS *authoring page* to
    place field objects (Legos, shovel, mosaic — only mosaic needs color), save
    them; listen for robot grab/place/drop and move the object to an "on-robot"
    sidebar. Wants a NEW page + persistence + a held-objects sidebar.
  - **#23 Log Replay System (LRS)** (P3, blocked/deferred): log encoder+sensor
    data on-robot, replay tick-by-tick on the computer with numbers + viz. Reuses
    the CTPS player; player_html.py already anticipates an "LRS-shaped track dict".
- Related-but-not-the-two: #55 (animate pivot arc — a canvas rendering detail,
  not a UI-layout reservation), #33 (CTPS epic parent), #83 (field-editor
  claude.ai-elegant restyle — sibling tool, aesthetic reference).

**Clarified framing:** Deliverable now is a *functional scoping spec* for a
refactored CTPS UI/UX — counts of **things / pages / abilities / functions** +
constraints + reserved hooks — to hand to a design tool ("Clot/Claude Design"),
which then hands a visual design back for coding. Fully spec the #125 player
refactor; RESERVE UI/UX spots (placeholder layout + wiring hooks, not a full
build) for all three related issues: #57 (object-authoring + on-robot held-items
sidebar), #55 (pivot-arc turn render mode), #23 (LRS real encoder/sensor
telemetry, tick-by-tick replay + source toggle). Page/file model is NOT strictly
limited — design may propose multiple pages/views — but the offline single-file
player (ADR-033/TC5) is the current contract, so relaxing it is a flagged
tradeoff needing a new ADR. Functional scope only; the design tool owns visuals.

## Perspectives (round 1)

### Critic (load-bearing points)
- All three reserved features need UPSTREAM data the trace doesn't carry today:
  #57 → no object identity/held-state/position (manip events are opaque string
  labels in track_builder ~L505); #23 → keyframe is only `{t,x,y,h}` on a
  SYNTHETIC constant-velocity clock, no slot for sensor numbers; #55 → turns are
  axle-center heading deltas, no pivot-wheel/radius recorded. => reserve as
  documented SEAMS, not phantom empty panels that rot and steal layout.
- Emitter tests are a wall of negative string asserts (`"src=" not in html`,
  no href/http/https, literal `id="ctps-play/seek/time"`, byte round-trip TRACK,
  whitespace-brittle poseAt regex). A design tool WILL emit `<img src>`,
  `<use href>`, web-font `<link>`, inline styles — all hard failures. The
  forbidden idiom must be a first-class part of the spec.
- Single-file/offline is load-bearing for BOTH the test harness and the
  field-day USB-stick story; relaxing it = a rewrite, not a knob.
- Backward-seek narration is already O(n) (`rebuildLog`); every new stateful
  panel must rebuild on each backward scrub — sits on the bounded-seek invariant.

### Feasibility — VERDICT: Conditional (no hard blockers)
- Multi-view = DOM show/hide inside ONE inlined file (vanilla JS), NOT multi-page.
- #57 "save" MUST be Blob + `<a download>` / `<input type=file>`+FileReader —
  file:// blocks File System Access API (SecurityError on opaque origin) and
  localStorage is undefined/unreliable there.
- #23 constant-velocity vs tick telemetry: NO conflict. Data Model B is already
  source-agnostic (TC6 renders a hand-built LRS dict). LRS supplies its own
  real-timestamp keyframes + an ADDITIVE per-tick numeric channel array; playhead
  still runs on `t`, readout binds to nearest sample via bisect (like findEventIndex).
- Reserved hooks need NO schema change now: Track has `schema_version`, player
  reads defensively (`keyframes||[]`, guarded obstacles, MAP_IMAGE null-fallback),
  emitter round-trips verbatim => unknown top-level keys survive. Reserve = disabled
  panels + wiring stubs + additive keys later. Document schema_version as additive-tolerant.
- Maintainability: author player.js as several asset files CONCATENATED at render
  time in `player_html.render` — modular source, still one HTML output.

### Thinker (branches — full list in task output)
1. Timeline-centric layout (multi-lane timeline IS the app)
2. Multi-panel dockable dashboard (region registry — reserving = "one more slot")
3. Inspector-driven "select anything" (nodes/edges/robot/objects/channels)
4. Transport as a first-class subsystem (speed/step/jump + full keymap)
5. Trace-source switcher: predicted vs replayed + ghost overlay (reserves #23)
6. Field-authoring as a second app MODE + held-items rail (reserves #57)
7. Robot render-mode registry (lerp default; wheel-arc #55; telemetry-pose later)
8. Reframe: don't refactor the player — define an EXTENSION CONTRACT (mount points)
9. Panel-less HUD/kiosk overlay (speculative)
10. Numbers-forward telemetry view w/ sparklines (reserves #23) (speculative)

## Live direction
**Extension-contract core (branch 8) realized as a single-file, multi-view vanilla-JS
app.** Player view built now (#125); Author view reserved (#57). Shared chrome:
layered canvas stage + render-mode registry (reserves #55), multi-lane timeline dock,
transport subsystem, select-anything inspector, event log. Three reservations land as
DOCUMENTED SEAMS + collapsible/stubbed slots + additive schema keys (NOT always-empty
panels): trace-source adapter + telemetry panel (#23), object store + held-items rail +
download/upload persistence (#57), render-mode registry (#55). Deliverable = the
functional scoping spec (counts) below, handed to the design tool.

## Killed / parked
- Multi-FILE / bundler / ES-module structure — parked: breaks TC5 round-trip
  extraction, the no-src/href asserts, and the offline field-day story. Single file
  with DOM view-switching instead. Relaxing needs a new ADR.
- Auto-save / localStorage for #57 — killed: unreliable on file://; use download/upload.
- Always-empty "reserved" panels stealing prime layout — killed per Critic; reserve
  as hidden/collapsible slots behind documented mount points instead.

## Decisions & debate verdicts
- **Single-file vs multi-file? → keep single-file OUTPUT, modularize the SOURCE.**
  The user's binary was a false dichotomy: source-organization and output-format
  are separable.
  - A) status-quo monolith (one player.js) — unmaintainable as it grows.
  - B) modular source (many assets/*.js,*.css) concatenated + inlined at emit time
    by player_html.render() → ONE self-contained output. **CHOSEN.** Keeps offline
    file:// field-day + all emitter tests (no src/href, TRACK byte round-trip) +
    NO ADR change (033's "self-contained" clause is about the shipped artifact).
    render() already inlines player.js today — extend to a list.
  - C) true multi-file output (ES modules / dev server) — REJECTED now: ES modules
    don't load over file:// (CORS), so it needs a server = kills field-day; breaks
    emitter tests; needs an ADR superseding 033; none of #57/#55/#23 require it.
  - Trigger to revisit C: a LIVE robot-telemetry stream (websocket), libs too heavy
    to inline, or genuine multipage routing — its own ADR at that point. (#23 LRS is
    a REPLAY of a recorded log = same Track ingest, not a live stream.)
