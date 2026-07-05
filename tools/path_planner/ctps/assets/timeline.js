/* CTPS — Clear Trajectory Prediction Simulator: timeline dock + lane registry.
 *
 * Owns the bottom TASK / GOTO / HOLDING / DRIFT dock: registers the four
 * `timelineLane` descriptors on `CTPS.timelineLanes` (module load time, so
 * PLAYER_META's lane manifest always finds an implementation) and, once at
 * boot, builds the dock DOM from those descriptors plus the ingested track:
 *
 *   1. Lane registration — `build(track)` for each lane returns an array of
 *      `{t0, t1?, label, cls}` items. An item WITH `t1` renders as a
 *      `.ctps-span` (a duration box); an item withOUT `t1` renders as a
 *      `.ctps-pin` (a point marker). This t1-presence check is the single
 *      discriminator — it works for `goto` (spans AND fault pins in the same
 *      lane) and `drift` (a pin plus a span in the same lane) without a
 *      separate per-item "kind" field.
 *   2. DOM: a 52px label column (`.ctps-lane-label`) and a lane column
 *      (`.ctps-lane`) per registered lane, in registration order (task, goto,
 *      holding, drift), plus a mono ruler computed from the track's actual
 *      `duration_s` (nice tick steps, never a fixed 0-90 window). Spans/pins
 *      are absolutely-positioned children sized in percent of `duration_s`,
 *      written only through `el.style.setProperty('left', ...)` /
 *      `setProperty('width', ...)` — the ADR-053 carve-out; this file never
 *      writes `el.style.<prop> =` directly, and never touches `--ctps-frac`
 *      (T5 owns writing it; the playhead div already in the template is
 *      positioned purely by CSS reading that custom property, so this file
 *      does not touch it at all).
 *   3. Drag-to-scrub: a pointer-capture drag across the `.ctps-timeline` dock
 *      resolves a fraction from the ruler's own track width (excluding the
 *      label gutter) and calls `store.seek(frac * duration)`.
 *
 * Coloring is class-only: lane containers/labels get a `--<id>` modifier
 * class, and `goto`/`drift` items get their own span/pin modifier classes
 * (`task` renders with the bare `.ctps-span`/`.ctps-pin` look). Actual colors
 * live in components.css (T13) — this file never sets `.style.color` or any
 * other inline color.
 *
 * Concatenated into its own <script> element AFTER player.js + store.js (so
 * `window.CTPS`, `CTPS.timelineLanes` and `CTPS.store` already exist) and
 * BEFORE boot.js (which runs the `init` this file pushes onto `CTPS._inits`).
 * Vanilla JS only — no frameworks, no network, works offline.
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
     * HOLDING lane (RESERVED — issue #57 seam): no cargo events exist yet, so
     * this always returns an empty array. The lane is still registered (and
     * still drawn as an empty row with its label) so the dock's layout and
     * the lane-order contract (task, goto, holding, drift) hold today and a
     * future cargo-episode implementation is purely additive here.
     */
    function buildHoldingLane(track) {
        return [];
    }

    /**
     * DRIFT lane: each `drift_warn` event becomes a point pin AT its time,
     * plus a span from that same time to end-of-track — the "uncorrected"
     * span, since nothing in the mission corrects drift after the warning.
     * Both items share the event's own label text.
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
                t1: duration,
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
        id: "holding",
        label: "HOLDING",
        kind: "span",
        color: "--ctps-hold",
        build: buildHoldingLane,
    });
    CTPS.timelineLanes.register({
        id: "drift",
        label: "DRIFT",
        kind: "point+span",
        color: "--ctps-drift",
        build: buildDriftLane,
    });

    // ----- ruler: nice tick steps from the ACTUAL duration_s ----------------

    // Candidate step sizes (seconds); the smallest candidate that keeps the
    // tick count at or below TARGET_TICK_COUNT wins. Covers both very short
    // traces (sub-4s) and long ones (tens of minutes) without a fixed window.
    var TICK_STEP_CANDIDATES = [
        0.5, 1, 2, 5, 10, 15, 20, 30, 60, 120, 300, 600, 900, 1800, 3600,
    ];
    var TARGET_TICK_COUNT = 8;

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

    /** Format a ruler tick label: whole seconds bare, fractional steps to 1dp. */
    function formatTickLabel(t) {
        var rounded = Math.round(t * 100) / 100;
        if (Math.abs(rounded - Math.round(rounded)) < 1e-6) {
            return String(Math.round(rounded)) + "s";
        }
        return rounded.toFixed(1) + "s";
    }

    /**
     * Populate `#ctps-ruler` with a 52px label gutter (reusing
     * `.ctps-lane-label` so it lines up with the lane rows below) and a tick
     * track. Returns the tick-track element — the drag-to-scrub geometry
     * reference, since it spans exactly the time axis (no label gutter).
     */
    function buildRulerDom(rulerEl, duration) {
        while (rulerEl.firstChild) {
            rulerEl.removeChild(rulerEl.firstChild);
        }

        var gutterEl = document.createElement("div");
        gutterEl.className = "ctps-lane-label";
        rulerEl.appendChild(gutterEl);

        var trackEl = document.createElement("div");
        trackEl.className = "ctps-ruler-track";
        rulerEl.appendChild(trackEl);

        var step = niceTickStep(duration);
        if (duration > 0 && step > 0) {
            var epsilon = step / 1000;
            for (var t = 0; t <= duration + epsilon; t += step) {
                appendTick(trackEl, t, duration);
            }
        } else {
            appendTick(trackEl, 0, duration);
        }
        return trackEl;
    }

    function appendTick(trackEl, t, duration) {
        var tickEl = document.createElement("div");
        tickEl.className = "ctps-ruler-tick";
        tickEl.style.setProperty("left", pctOf(t, duration) + "%");
        tickEl.textContent = formatTickLabel(t);
        trackEl.appendChild(tickEl);
    }

    // ----- lanes DOM ----------------------------------------------------------

    // Task/goto spans (element + its [t0, t1] window) recorded at build time so
    // the playhead subscription can light up whichever one the current time
    // falls inside. Rebuilt each buildLanesDom(); other lane kinds are excluded.
    var _currentSpans = [];

    /**
     * Toggle `.ctps-span--current` on each recorded task/goto span so the one
     * the playhead time `t` sits inside gets the theme-accent highlight and the
     * rest stay neutral gray. Inclusive on both ends, so a span reads as current
     * from the instant its command starts through its end boundary.
     */
    function refreshCurrent(t) {
        for (var i = 0; i < _currentSpans.length; i++) {
            var s = _currentSpans[i];
            if (t >= s.t0 && t <= s.t1) {
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
     * Populate `#ctps-lanes` with one label + lane pair per registered
     * `timelineLane`, in registration order (task, goto, holding, drift). A
     * lane whose `build` throws is skipped (recorded into `CTPS._errors`)
     * rather than aborting the whole dock — the same "one bad module can't
     * take down the page" posture the registries themselves use.
     */
    function buildLanesDom(lanesEl, lanes, track, duration) {
        while (lanesEl.firstChild) {
            lanesEl.removeChild(lanesEl.firstChild);
        }
        _currentSpans.length = 0;

        for (var i = 0; i < lanes.length; i++) {
            var lane = lanes[i];

            var labelEl = document.createElement("div");
            labelEl.className = "ctps-lane-label ctps-lane-label--" + lane.id;
            labelEl.textContent = lane.label;
            lanesEl.appendChild(labelEl);

            var laneEl = document.createElement("div");
            laneEl.className = "ctps-lane ctps-lane--" + lane.id;
            lanesEl.appendChild(laneEl);

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

    // ----- drag-to-scrub -------------------------------------------------------

    /**
     * Wire pointer drag-to-scrub over the whole `.ctps-timeline` dock. The
     * fraction is resolved from `trackEl` (the ruler's tick track) so the
     * label gutter never biases the mapping, even when the pointer is over a
     * lane row rather than the ruler itself — `clientX` is compared against
     * `trackEl`'s own bounding rect regardless of which descendant received
     * the event. Pointer capture keeps `pointermove` firing while dragging
     * outside the dock's bounds.
     */
    function wireScrub(timelineEl, trackEl, store, duration) {
        var dragging = false;

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
            store.seek(frac * duration);
        }

        timelineEl.addEventListener("pointerdown", function (evt) {
            dragging = true;
            if (timelineEl.setPointerCapture) {
                try {
                    timelineEl.setPointerCapture(evt.pointerId);
                } catch (captureError) {
                    // Pointer capture is best-effort; dragging still works
                    // via plain pointermove if capture is unavailable.
                }
            }
            seekFromClientX(evt.clientX);
        });

        timelineEl.addEventListener("pointermove", function (evt) {
            if (dragging) {
                seekFromClientX(evt.clientX);
            }
        });

        function stopDrag() {
            dragging = false;
        }
        timelineEl.addEventListener("pointerup", stopDrag);
        timelineEl.addEventListener("pointercancel", stopDrag);
    }

    // ----- boot wiring ----------------------------------------------------------

    /**
     * Build the dock DOM once from the registered lanes + the ingested track,
     * then wire drag-to-scrub and a lightweight playhead subscription. Lane
     * geometry is static for the life of the page (events don't change during
     * playback) and the playhead LINE is positioned purely by CSS reading
     * `--ctps-frac` (written by T5); the one thing that reacts per `t` is the
     * `.ctps-span--current` highlight on the task/goto span under the playhead.
     */
    function init(ctx) {
        var timelineEl = document.querySelector(".ctps-timeline");
        var rulerEl = document.getElementById("ctps-ruler");
        var lanesEl = document.getElementById("ctps-lanes");
        if (!timelineEl || !rulerEl || !lanesEl) {
            CTPS._errors.push("timeline: dock container not found in DOM");
            return;
        }

        try {
            var track = (ctx && ctx.track) || CTPS.engine.track;
            var duration = (track && track.duration_s) || 0;
            var store = (ctx && ctx.store) || CTPS.store;

            var rulerTrackEl = buildRulerDom(rulerEl, duration);
            buildLanesDom(lanesEl, CTPS.timelineLanes.all(), track, duration);
            if (store) {
                wireScrub(timelineEl, rulerTrackEl, store, duration);
                // React to the playhead: light up the task/goto span the current
                // time sits inside. `seek`/`setT` notify with ["t"] on every drag
                // and playback tick, so this stays in sync without its own clock.
                store.subscribe(function (changed) {
                    if (changed && changed.indexOf("t") !== -1) {
                        refreshCurrent(store.t);
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
