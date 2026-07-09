/* CTPS — Clear Trajectory Prediction Simulator: timeline dock + lane registry.
 *
 * Owns the bottom TASK / GOTO / MAGAZINE / REAR BOX / DRIFT dock: registers
 * the five `timelineLane` descriptors on `CTPS.timelineLanes` (module load
 * time, so PLAYER_META's lane manifest always finds an implementation) and,
 * once at boot, builds the dock DOM from those descriptors plus the track:
 *
 *   1. Lane registration — `build(track)` for each lane returns an array of
 *      `{t0, t1?, label, cls}` items. An item WITH `t1` renders as a
 *      `.ctps-span` (a duration box); an item withOUT `t1` renders as a
 *      `.ctps-pin` (a point marker). This t1-presence check is the single
 *      discriminator — it works for `goto` (spans AND fault pins in the same
 *      lane) and `drift` (a pin plus a span in the same lane) without a
 *      separate per-item "kind" field.
 *   2. DOM: one `.ctps-tl-row` per registered lane, in registration order
 *      (task, goto, magazine, rearbox, drift) — each row holds a
 *      `.ctps-lane-label` gutter cell + a `.ctps-lane` track, as flex
 *      siblings — plus one more `.ctps-tl-row` for the ruler, whose track
 *      element is `#ctps-ruler-track`: built ONCE at init and stable for the
 *      page life (see point 4). Spans/pins/ticks stay absolutely-positioned
 *      children sized in percent of `duration_s` — unchanged by the
 *      zoom/pan work, since the track element is just `--ctps-content-w`
 *      wide now instead of the viewport width — written only through
 *      `el.style.setProperty('left', ...)` / `setProperty('width', ...)` —
 *      the ADR-053 carve-out; this file never writes `el.style.<prop> =`
 *      directly, and never touches `--ctps-frac` (T5 owns writing it; the
 *      playhead div already in the template is positioned purely by CSS
 *      reading that custom property, so this file does not touch it at
 *      all).
 *   3. Drag-to-scrub: a pointer-capture drag over `#ctps-tl-viewport` (NOT
 *      the toolbar, the sticky labels, or the custom scrollbar — those are
 *      walked out via `closest()` from the pointerdown target and ignored)
 *      resolves a fraction from the stable `#ctps-ruler-track` element's own
 *      `getBoundingClientRect()` and calls `store.seek(frac * duration)`.
 *      Reading the rect straight off the live element keeps the mapping
 *      correct at any scroll offset or zoom level, since the rect reflects
 *      rendered geometry — there is no cached width/offset to go stale.
 *   4. Windowed ruler ticks: `CTPS.timeline.rebuildRuler(pxPerSec, viewT0,
 *      viewT1)` repaints ONLY the tick children within `[viewT0, viewT1]`
 *      (plus a one-screen margin on each side), at a step chosen from
 *      `TICK_STEP_CANDIDATES` so on-screen spacing stays ~90-120px for the
 *      given `pxPerSec` — `pxPerSec` is the SOLE density input; this file
 *      never reads `--ctps-content-w` back via `getComputedStyle`. It never
 *      recreates `#ctps-ruler-track` itself, only its tick children, so
 *      drag-to-scrub's captured rect reference never goes stale.
 *      `timeline_view.js` (T3) calls this after every committed zoom/pan;
 *      `init()` below also paints an initial full-duration window up front,
 *      so ticks are visible before any zoom/pan module has run — or if one
 *      never successfully does (matching boot.js's per-module isolation).
 *
 * Coloring is class-only: lane containers/labels get a `--<id>` modifier
 * class, and `goto`/`drift` items get their own span/pin modifier classes
 * (`task` renders with the bare `.ctps-span`/`.ctps-pin` look). Actual colors
 * live in components.css (T13) — this file never sets `.style.color` or any
 * other inline color.
 *
 * Concatenated into its own <script> element AFTER player.js + store.js (so
 * `window.CTPS`, `CTPS.timelineLanes` and `CTPS.store` already exist) and
 * BEFORE boot.js (which runs the `init` this file pushes onto `CTPS._inits`)
 * — and BEFORE timeline_view.js (T3), which reads the rows this file builds
 * and calls `CTPS.timeline.rebuildRuler`. Vanilla JS only — no frameworks,
 * no network, works offline.
 */

"use strict";

(function () {
    var CTPS = window.CTPS;
    if (!CTPS || !CTPS.timelineLanes) {
        // player.js failed to create the namespace before this file ran —
        // nothing to register against and nowhere to record an error either.
        return;
    }

    // ----- shared percent-of-duration helper -------------------------------

    /**
     * Percent position of time `t` along a track of length `duration`,
     * clamped into [0, 100]. Returns 0 for a zero/negative/absent duration
     * (an empty or degenerate trace) rather than dividing by zero.
     */
    function pctOf(t, duration) {
        if (!duration || duration <= 0) {
            return 0;
        }
        var pct = (t / duration) * 100;
        if (pct < 0) {
            pct = 0;
        } else if (pct > 100) {
            pct = 100;
        }
        return pct;
    }

    // ----- lane builders -----------------------------------------------------

    /**
     * TASK lane: pair `task_start` -> `task_end` events into spans labelled
     * with the task name (the event's own `label`, per track_builder). Uses a
     * stack so sequential (non-reentrant) tasks pair correctly; a start left
     * unmatched at track end (e.g. a fault mid-task) is closed out to
     * `duration_s` rather than dropped.
     */
    function buildTaskLane(track) {
        var events = (track && track.events) || [];
        var duration = (track && track.duration_s) || 0;
        var out = [];
        var open = [];
        for (var i = 0; i < events.length; i++) {
            var ev = events[i];
            if (ev.type === "task_start") {
                open.push({ t0: ev.t, label: ev.label });
            } else if (ev.type === "task_end") {
                var start = open.pop();
                if (start) {
                    out.push({
                        t0: start.t0,
                        t1: ev.t,
                        label: start.label,
                        cls: "ctps-span--task",
                    });
                }
            }
        }
        for (var j = 0; j < open.length; j++) {
            out.push({
                t0: open[j].t0,
                t1: duration,
                label: open[j].label,
                cls: "ctps-span--task",
            });
        }
        return out;
    }

    /**
     * GOTO lane (LIVE): a successful `goto` event carries `t_end` and renders
     * as a span `t -> t_end` labelled `from->target` (the event's own
     * `label`, emitted verbatim by track_builder). An UNREACHABLE fault goto
     * has NO `t_end` and a label starting "UNREACHABLE ..." — it renders as a
     * point pin styled as a fault. The `t_end` presence is the discriminator
     * (robust to label text); the UNREACHABLE convention is documented here
     * to match the Data Model.
     */
    function buildGotoLane(track) {
        var events = (track && track.events) || [];
        var out = [];
        for (var i = 0; i < events.length; i++) {
            var ev = events[i];
            if (ev.type !== "goto") {
                continue;
            }
            var hasEnd = (ev.t_end !== undefined && ev.t_end !== null);
            if (hasEnd) {
                // SUCCESS span: t -> t_end, label "from->target".
                out.push({
                    t0: ev.t,
                    t1: ev.t_end,
                    label: ev.label,
                    cls: "ctps-span--goto",
                });
            } else {
                // UNREACHABLE fault: no t_end, label starts "UNREACHABLE ...".
                out.push({
                    t0: ev.t,
                    label: ev.label,
                    cls: "ctps-pin--fault",
                });
            }
        }
        return out;
    }

    /**
     * True when the magazine holds at least one tile in ANY lane for one
     * resolved cargo scene snapshot — the predicate behind the MAGAZINE lane.
     * A rear-box Cargo is deliberately excluded: the rear box has its own lane.
     */
    function _magazineHeld(scene) {
        if (!scene) {
            return false;
        }
        var magazine = scene.magazine || [];
        for (var lane = 0; lane < magazine.length; lane++) {
            if (magazine[lane].length > 0) {
                return true;
            }
        }
        return false;
    }

    /** True when the rear box holds a Cargo this snapshot — the REAR BOX lane predicate. */
    function _rearBoxHeld(scene) {
        return !!(scene && scene.rearBox);
    }

    /**
     * Contiguous-span builder shared by both cargo lanes. Recomputes the
     * resolve-once scene timeline via `CTPS.cargo.resolve(track,
     * store.objects, store.robotCfg)` rather than reaching into store.js's
     * private `sceneTimeline` closure (not exposed) — cheap, since it runs once
     * per manip event, not per frame — then emits one span per contiguous
     * period for which `predicate(snapshot)` holds. Returns `[]` before any
     * layout is loaded (no objects => an empty resolve) or when the predicate
     * never holds.
     */
    function _cargoSpans(track, predicate, label, cls) {
        var store = CTPS.store;
        var resolve = CTPS.cargo && CTPS.cargo.resolve;
        if (!store || !resolve) {
            return [];
        }
        var duration = (track && track.duration_s) || 0;
        // Silent re-resolve: `store.loadLayout` already logged each no-op/error
        // once; these lane builders re-run per scrub boundary and must not repeat it.
        var timeline = resolve(track, store.objects, store.robotCfg, true);
        var out = [];
        var openT0 = null;
        for (var i = 0; i < timeline.length; i++) {
            var snap = timeline[i];
            var held = predicate(snap);
            if (held && openT0 === null) {
                openT0 = snap.t;
            } else if (!held && openT0 !== null) {
                out.push({ t0: openT0, t1: snap.t, label: label, cls: cls });
                openT0 = null;
            }
        }
        if (openT0 !== null) {
            out.push({ t0: openT0, t1: duration, label: label, cls: cls });
        }
        return out;
    }

    /**
     * MAGAZINE lane (issue #57): one `--ctps-hold` span per contiguous period
     * during which the magazine holds at least one tile — mirroring the cargo
     * panel's Magazine tab and the HUD's `magazine N/CAP` count (cargo.js). The
     * spans carry an EMPTY label: the lane's own "MAGAZINE" gutter label already
     * names it, so on-bar text would just repeat it.
     */
    function buildMagazineLane(track) {
        return _cargoSpans(track, _magazineHeld, "", "ctps-span--magazine");
    }

    /**
     * REAR BOX lane (issue #57): one `--ctps-hold` span per contiguous period
     * during which the rear box holds a Cargo — mirroring the Inventory tab and
     * the HUD's `rear box N/CAP` count (cargo.js). Empty span label, same reason
     * as the magazine lane (the gutter label already reads "REAR BOX").
     */
    function buildRearBoxLane(track) {
        return _cargoSpans(track, _rearBoxHeld, "", "ctps-span--rearbox");
    }

    /**
     * DRIFT lane: each `drift_warn` event becomes a point pin AT its time (the
     * off-plan move that opened the drift) plus a span from that time to its
     * `t_end` — the goto re-anchor that corrected it. An event with no `t_end`
     * (an uncorrected drift that ran to the end, or a legacy/LRS-shaped point)
     * spans to end-of-track. Both items share the event's own label text.
     */
    function buildDriftLane(track) {
        var events = (track && track.events) || [];
        var duration = (track && track.duration_s) || 0;
        var out = [];
        for (var i = 0; i < events.length; i++) {
            var ev = events[i];
            if (ev.type !== "drift_warn") {
                continue;
            }
            out.push({ t0: ev.t, label: ev.label, cls: "ctps-pin--drift" });
            out.push({
                t0: ev.t,
                t1: ev.t_end != null ? ev.t_end : duration,
                label: ev.label,
                cls: "ctps-span--drift",
            });
        }
        return out;
    }

    // ----- registration (module load time, order = render order) -----------

    CTPS.timelineLanes.register({
        id: "task",
        label: "TASK",
        kind: "span",
        color: "--ctps-text-2",
        build: buildTaskLane,
    });
    CTPS.timelineLanes.register({
        id: "goto",
        label: "GOTO",
        kind: "span",
        color: "--ctps-text-2",
        build: buildGotoLane,
    });
    CTPS.timelineLanes.register({
        id: "magazine",
        label: "MAGAZINE",
        kind: "span",
        color: "--ctps-hold",
        build: buildMagazineLane,
    });
    CTPS.timelineLanes.register({
        id: "rearbox",
        label: "REAR BOX",
        kind: "span",
        color: "--ctps-hold",
        build: buildRearBoxLane,
    });
    CTPS.timelineLanes.register({
        id: "drift",
        label: "DRIFT",
        kind: "point+span",
        color: "--ctps-drift",
        build: buildDriftLane,
    });

    // ----- ruler: nice tick steps + windowed density-based repaint ----------

    // Candidate step sizes (seconds). The initial full-duration paint (at
    // init, before any zoom/pan module has run) picks the smallest candidate
    // that keeps the tick count at or below TARGET_TICK_COUNT across the
    // WHOLE track — covers both very short traces (sub-4s) and long ones
    // (tens of minutes) without a fixed window. Once zoomed, `rebuildRuler`
    // picks from this SAME list by on-screen pixel spacing instead (see
    // `tickStepForDensity`) so both paths agree on "nice" step values.
    var TICK_STEP_CANDIDATES = [
        0.5, 1, 2, 5, 10, 15, 20, 30, 60, 120, 300, 600, 900, 1800, 3600,
    ];
    var TARGET_TICK_COUNT = 8;

    // Target on-screen spacing (px) between ticks once zoomed; the smallest
    // candidate step whose pixel spacing at the given density reaches this
    // threshold wins, landing ticks in the ~90-120px band the plan calls for.
    var TARGET_TICK_PX = 100;

    /** Return a "nice" tick step (seconds) for a track of length `duration`. */
    function niceTickStep(duration) {
        if (!duration || duration <= 0) {
            return 0;
        }
        var raw = duration / TARGET_TICK_COUNT;
        for (var i = 0; i < TICK_STEP_CANDIDATES.length; i++) {
            if (TICK_STEP_CANDIDATES[i] >= raw) {
                return TICK_STEP_CANDIDATES[i];
            }
        }
        return TICK_STEP_CANDIDATES[TICK_STEP_CANDIDATES.length - 1];
    }

    /**
     * Return a "nice" tick step (seconds) so ticks land ~`TARGET_TICK_PX`
     * apart on screen at `pxPerSec` (pixels per second of track). This is the
     * SOLE density input for windowed repaints — `rebuildRuler` never reads
     * `--ctps-content-w` back via `getComputedStyle`. A non-positive
     * `pxPerSec` (not yet known, or a degenerate track) falls back to the
     * finest candidate.
     */
    function tickStepForDensity(pxPerSec) {
        if (!pxPerSec || pxPerSec <= 0) {
            return TICK_STEP_CANDIDATES[0];
        }
        for (var i = 0; i < TICK_STEP_CANDIDATES.length; i++) {
            var candidate = TICK_STEP_CANDIDATES[i];
            if (candidate * pxPerSec >= TARGET_TICK_PX) {
                return candidate;
            }
        }
        return TICK_STEP_CANDIDATES[TICK_STEP_CANDIDATES.length - 1];
    }

    /** Format a ruler tick label: whole seconds bare, fractional steps to 1dp. */
    function formatTickLabel(t) {
        var rounded = Math.round(t * 100) / 100;
        if (Math.abs(rounded - Math.round(rounded)) < 1e-6) {
            return String(Math.round(rounded)) + "s";
        }
        return rounded.toFixed(1) + "s";
    }

    /**
     * Build the ruler's `.ctps-tl-row` DOM ONCE — a `.ctps-lane-label`
     * gutter cell (so it lines up with the lane rows below) plus the
     * `#ctps-ruler-track` track element. Returns the (stable) track element;
     * no tick children are appended here — that is `paintRulerTicks`'s job,
     * kept separate so `rebuildRuler` (T3's zoom/pan hook) can repaint ticks
     * without ever recreating the track element itself. Recreating it would
     * invalidate `wireScrub`'s captured rect reference — the recorded
     * stable-ruler-track gotcha.
     */
    function buildRulerDom(rulerEl) {
        while (rulerEl.firstChild) {
            rulerEl.removeChild(rulerEl.firstChild);
        }

        var rowEl = document.createElement("div");
        rowEl.className = "ctps-tl-row";
        rulerEl.appendChild(rowEl);

        var gutterEl = document.createElement("div");
        gutterEl.className = "ctps-lane-label";
        rowEl.appendChild(gutterEl);

        var trackEl = document.createElement("div");
        trackEl.className = "ctps-ruler-track";
        trackEl.id = "ctps-ruler-track";
        rowEl.appendChild(trackEl);

        return trackEl;
    }

    /**
     * Clear and repaint ONLY `trackEl`'s tick children, at `step` seconds,
     * for ticks whose time falls in `[rangeLo, rangeHi]` (both clamped into
     * `[0, duration]`). Ticks stay positioned by `pctOf(t, duration)` — a
     * percent of the (now `--ctps-content-w` wide) track — so this function
     * never needs the track's pixel width. A non-positive `duration`/`step`
     * falls back to a single tick at `t=0`, matching the prior degenerate-
     * track behavior.
     */
    function paintRulerTicks(trackEl, duration, step, rangeLo, rangeHi) {
        while (trackEl.firstChild) {
            trackEl.removeChild(trackEl.firstChild);
        }
        if (!duration || duration <= 0 || !step || step <= 0) {
            appendTick(trackEl, 0, duration);
            return;
        }
        var lo = rangeLo < 0 ? 0 : rangeLo;
        var hi = rangeHi > duration ? duration : rangeHi;
        var epsilon = step / 1000;
        var firstIndex = Math.floor(lo / step);
        if (firstIndex < 0) {
            firstIndex = 0;
        }
        for (var t = firstIndex * step; t <= hi + epsilon; t += step) {
            if (t >= lo - epsilon) {
                appendTick(trackEl, t, duration);
            }
        }
    }

    function appendTick(trackEl, t, duration) {
        var tickEl = document.createElement("div");
        tickEl.className = "ctps-ruler-tick";
        tickEl.style.setProperty("left", pctOf(t, duration) + "%");
        tickEl.textContent = formatTickLabel(t);
        trackEl.appendChild(tickEl);
    }

    // Stable ruler track element + its track's duration, captured once at
    // init() so `rebuildRuler` (called by T3 well after init, on every
    // committed zoom/pan) can repaint tick children without either being
    // passed back in.
    var _rulerTrackEl = null;
    var _rulerDuration = 0;

    /**
     * `CTPS.timeline.rebuildRuler` — repaint ONLY the ruler's tick children
     * within the visible window `[viewT0, viewT1]` plus a one-screen margin
     * on each side (bounds tick node count on long traces while giving a
     * smooth pan some slack before ticks pop in right at the edge), at a
     * step chosen from `TICK_STEP_CANDIDATES` for ~90-120px on-screen
     * spacing at `pxPerSec`. `pxPerSec` is the sole density input — never
     * `--ctps-content-w` read back via `getComputedStyle`. Never recreates
     * `#ctps-ruler-track`, only its tick children. A no-op before `init()`
     * has run (no stable track element yet).
     */
    function rebuildRuler(pxPerSec, viewT0, viewT1) {
        if (!_rulerTrackEl) {
            return;
        }
        var step = tickStepForDensity(pxPerSec);
        var span = viewT1 - viewT0;
        if (span < 0) {
            span = 0;
        }
        var margin = span;
        paintRulerTicks(
            _rulerTrackEl, _rulerDuration, step, viewT0 - margin, viewT1 + margin);
    }

    // Exposed at module load time (this file runs before timeline_view.js,
    // per _ASSET_JS order) so `CTPS.timeline.rebuildRuler` always exists by
    // the time T3 looks for it, even though repaints are inert until
    // `init()` below sets `_rulerTrackEl`.
    CTPS.timeline = { rebuildRuler: rebuildRuler };

    // ----- lanes DOM ----------------------------------------------------------

    // Task/goto spans (element + its [t0, t1] window) recorded at build time so
    // the playhead subscription can light up whichever one the current time
    // falls inside. Rebuilt each buildLanesDom(); other lane kinds are excluded.
    var _currentSpans = [];

    /**
     * Toggle `.ctps-span--current` on each recorded task/goto span so the one
     * the playhead time `t` sits inside gets the theme-accent highlight and the
     * rest stay neutral gray. Half-open on the upper bound (`t0 <= t < t1`): when
     * two same-lane spans share a boundary (goto A's end == goto B's start) only
     * the later span is current at that instant, never both. A goto span nested
     * inside its task span still lights both — they are independent spans in
     * different lanes, not a shared-boundary tie.
     */
    function refreshCurrent(t) {
        for (var i = 0; i < _currentSpans.length; i++) {
            var s = _currentSpans[i];
            if (t >= s.t0 && t < s.t1) {
                s.el.classList.add("ctps-span--current");
            } else {
                s.el.classList.remove("ctps-span--current");
            }
        }
    }

    /**
     * Render one lane's built items into `laneEl` as absolutely-positioned
     * `.ctps-span` (has `t1`) / `.ctps-pin` (no `t1`) children, each sized in
     * percent of `duration` and carrying the item's own `cls` (if any) on top
     * of the base class. Geometry is written only via `setProperty`. Task/goto
     * spans are also recorded into `_currentSpans` for the playhead highlight.
     */
    function renderLaneItems(laneEl, items, duration) {
        for (var i = 0; i < items.length; i++) {
            var item = items[i];
            var isSpan = (item.t1 !== undefined && item.t1 !== null);
            var el = document.createElement("div");
            el.className = (isSpan ? "ctps-span" : "ctps-pin") +
                (item.cls ? " " + item.cls : "");
            if (item.label) {
                el.title = item.label;
                if (isSpan) {
                    el.textContent = item.label;
                }
            }
            var leftPct = pctOf(item.t0, duration);
            el.style.setProperty("left", leftPct + "%");
            if (isSpan) {
                var widthPct = pctOf(item.t1, duration) - leftPct;
                if (widthPct < 0) {
                    widthPct = 0;
                }
                el.style.setProperty("width", widthPct + "%");
            }
            if (isSpan &&
                (item.cls === "ctps-span--task" || item.cls === "ctps-span--goto")) {
                _currentSpans.push({ el: el, t0: item.t0, t1: item.t1 });
            }
            laneEl.appendChild(el);
        }
    }

    /**
     * Populate `#ctps-lanes` with one `.ctps-tl-row` (a label cell + lane
     * track, as flex siblings) per registered `timelineLane`, in
     * registration order (task, goto, magazine, rearbox, drift). A lane
     * whose `build` throws is skipped (recorded into `CTPS._errors`) rather
     * than aborting the whole dock — the same "one bad module can't take
     * down the page" posture the registries themselves use.
     */
    function buildLanesDom(lanesEl, lanes, track, duration) {
        while (lanesEl.firstChild) {
            lanesEl.removeChild(lanesEl.firstChild);
        }
        _currentSpans.length = 0;

        for (var i = 0; i < lanes.length; i++) {
            var lane = lanes[i];

            var rowEl = document.createElement("div");
            rowEl.className = "ctps-tl-row";
            lanesEl.appendChild(rowEl);

            var labelEl = document.createElement("div");
            labelEl.className = "ctps-lane-label ctps-lane-label--" + lane.id;
            labelEl.textContent = lane.label;
            rowEl.appendChild(labelEl);

            var laneEl = document.createElement("div");
            laneEl.className = "ctps-lane ctps-lane--" + lane.id;
            rowEl.appendChild(laneEl);

            var items = [];
            try {
                items = lane.build(track) || [];
            } catch (buildError) {
                CTPS._errors.push(
                    "timeline: lane '" + lane.id + "' build() threw: " +
                    (buildError && buildError.message || String(buildError)));
                items = [];
            }
            renderLaneItems(laneEl, items, duration);
        }
    }

    /**
     * Rebuild ONLY the two cargo lanes' DOM in place (MAGAZINE + REAR BOX) —
     * never the whole dock via `buildLanesDom` (task/goto/drift don't depend on
     * cargo state, and a full rebuild would also reset `_currentSpans`'
     * task/goto bookkeeping). Called on every `["scene"]` store notify:
     * `store.loadLayout` (embedded OR uploaded) and the t->scene bisect
     * controller (store.js) both notify `"scene"`. The initial (empty) cargo
     * lanes are created by the normal dock build (`buildLanesDom`), NOT by this
     * function; this is the ONLY path that keeps them in sync after boot — the
     * dock otherwise builds its lanes once (see the file docstring), so without
     * this rebuild the cargo lanes stay permanently empty for embedded and
     * uploaded layouts alike.
     */
    function rebuildCargoLanesDom(track, duration) {
        _rebuildOneCargoLane(
            ".ctps-lane--magazine", buildMagazineLane, "magazine", track, duration);
        _rebuildOneCargoLane(
            ".ctps-lane--rearbox", buildRearBoxLane, "rearbox", track, duration);
    }

    /** Clear + repaint one cargo lane element from its builder, error-isolated. */
    function _rebuildOneCargoLane(selector, builder, id, track, duration) {
        var laneEl = document.querySelector(selector);
        if (!laneEl) {
            return;
        }
        while (laneEl.firstChild) {
            laneEl.removeChild(laneEl.firstChild);
        }
        var items = [];
        try {
            items = builder(track) || [];
        } catch (buildError) {
            CTPS._errors.push(
                "timeline: lane '" + id + "' rebuild threw: " +
                (buildError && buildError.message || String(buildError)));
            items = [];
        }
        renderLaneItems(laneEl, items, duration);
    }

    // ----- drag-to-scrub -------------------------------------------------------

    /**
     * Wire pointer drag-to-scrub over `#ctps-tl-viewport` (NOT the whole
     * `.ctps-timeline` dock — the toolbar and the custom scrollbar strip
     * live outside the viewport and so never reach this listener at all; the
     * sticky `.ctps-lane-label` gutter DOES live inside the viewport, so its
     * pointer-downs are explicitly ignored below via a `closest()` walk).
     * The fraction is resolved from `trackEl` (the stable `#ctps-ruler-track`
     * element) so the label gutter never biases the mapping and the mapping
     * stays correct at any scroll offset/zoom level — `clientX` is compared
     * against `trackEl`'s own live bounding rect regardless of which
     * descendant received the event. Pointer capture keeps `pointermove`
     * firing while dragging outside the viewport's bounds. The drag pauses
     * playback (like the `#ctps-seek` slider) so the rAF clock cannot
     * advance the playhead mid-scrub and fight the drag.
     */
    function wireScrub(viewportEl, trackEl, store, duration) {
        var dragging = false;

        function isIgnoredTarget(target) {
            if (!target || !target.closest) {
                return false;
            }
            return !!(
                target.closest(".ctps-lane-label") ||
                target.closest(".ctps-tl-toolbar") ||
                target.closest(".ctps-tl-scroll"));
        }

        function seekFromClientX(clientX) {
            var rect = trackEl.getBoundingClientRect();
            var frac = 0;
            if (rect.width > 0) {
                frac = (clientX - rect.left) / rect.width;
                if (frac < 0) {
                    frac = 0;
                } else if (frac > 1) {
                    frac = 1;
                }
            }
            if (store.playing) {
                store.setPlaying(false);
            }
            store.seek(frac * duration);
        }

        viewportEl.addEventListener("pointerdown", function (evt) {
            if (isIgnoredTarget(evt.target)) {
                return;
            }
            dragging = true;
            if (viewportEl.setPointerCapture) {
                try {
                    viewportEl.setPointerCapture(evt.pointerId);
                } catch (captureError) {
                    // Pointer capture is best-effort; dragging still works
                    // via plain pointermove if capture is unavailable.
                }
            }
            seekFromClientX(evt.clientX);
        });

        viewportEl.addEventListener("pointermove", function (evt) {
            if (dragging) {
                seekFromClientX(evt.clientX);
            }
        });

        function stopDrag() {
            dragging = false;
        }
        viewportEl.addEventListener("pointerup", stopDrag);
        viewportEl.addEventListener("pointercancel", stopDrag);
    }

    // ----- boot wiring ----------------------------------------------------------

    /**
     * Build the dock DOM once from the registered lanes + the ingested track,
     * then wire drag-to-scrub and a lightweight playhead subscription. Lane
     * geometry is static for the life of the page (events don't change during
     * playback) and the playhead LINE is positioned purely by CSS reading
     * `--ctps-frac` (written by T5); the one thing that reacts per `t` is the
     * `.ctps-span--current` highlight on the task/goto span under the
     * playhead. The ruler's track element (`#ctps-ruler-track`) is built
     * once here and its tick children painted once for the full duration —
     * `rebuildRuler` (T3's zoom/pan hook) repaints only those tick children
     * afterwards, never the track element itself.
     */
    function init(ctx) {
        var viewportEl = document.getElementById("ctps-tl-viewport");
        var rulerEl = document.getElementById("ctps-ruler");
        var lanesEl = document.getElementById("ctps-lanes");
        if (!viewportEl || !rulerEl || !lanesEl) {
            CTPS._errors.push("timeline: dock container not found in DOM");
            return;
        }

        try {
            var track = (ctx && ctx.track) || CTPS.engine.track;
            var duration = (track && track.duration_s) || 0;
            var store = (ctx && ctx.store) || CTPS.store;

            var rulerTrackEl = buildRulerDom(rulerEl);
            _rulerTrackEl = rulerTrackEl;
            _rulerDuration = duration;
            paintRulerTicks(rulerTrackEl, duration, niceTickStep(duration), 0, duration);

            buildLanesDom(lanesEl, CTPS.timelineLanes.all(), track, duration);
            if (store) {
                wireScrub(viewportEl, rulerTrackEl, store, duration);
                // React to the playhead: light up the task/goto span the current
                // time sits inside. `seek`/`setT` notify with ["t"] on every drag
                // and playback tick, so this stays in sync without its own clock.
                // `["scene"]` fires on layout-load (embedded or uploaded) and on
                // every cargo event crossed while scrubbing — rebuild the two
                // cargo lanes only, per rebuildCargoLanesDom's docstring.
                store.subscribe(function (changed) {
                    if (changed && changed.indexOf("t") !== -1) {
                        refreshCurrent(store.t);
                    }
                    if (changed && changed.indexOf("scene") !== -1) {
                        rebuildCargoLanesDom(track, duration);
                    }
                });
                refreshCurrent(store.t);
            }
        } catch (initError) {
            CTPS._errors.push(
                "timeline: init() threw: " +
                (initError && initError.message || String(initError)));
        }
    }

    CTPS._inits.push(init);
})();
