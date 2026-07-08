/* CTPS — Clear Trajectory Prediction Simulator: timeline zoom + move bar.
 *
 * Owns the bottom timeline dock's HORIZONTAL navigation — zoom and pan — so a
 * long trace reads like a video/DAW editor instead of cramming every span and
 * pin into a fixed 0..duration track. `timeline.js` still owns the lane/ruler
 * DOM (it builds the per-lane rows + the stable `#ctps-ruler-track`); this file
 * owns everything that MOVES that content:
 *
 *   1. Zoom state — a single factor `z` in `[1, zoomMax]`. `z = 1` is
 *      fit-to-width (the whole trace fills the visible track, no scroll);
 *      `zoomMax` is capped so a short window (~1.5 s) fills the view. The
 *      content track width is `fitContentW * z`, written to the single CSS
 *      length knob `--ctps-content-w` on `.ctps-timeline`.
 *   2. The zoom widget — `+` / `-` buttons, a log-mapped slider, a `Fit`
 *      button, and an `N.N×` readout, all kept two-way in sync.
 *   3. Panning — a custom themed scrollbar thumb (`--ctps-tl-thumb-w` /
 *      `--ctps-tl-thumb-x` on `#ctps-tl-thumb`) synced to `scrollLeft`,
 *      thumb-drag, `Ctrl`/`Cmd`+wheel cursor-anchored zoom, plain vertical-
 *      wheel horizontal pan, and native trackpad `deltaX` pass-through.
 *   4. Follow-the-playhead — during playback the view auto-pages to keep the
 *      playhead visible; a manual pan suspends it until the next play edge or
 *      a `fit()`.
 *   5. Re-anchoring — on zoom the playhead (buttons/slider/keys) or the point
 *      under the cursor (`Ctrl`/`Cmd`+wheel) stays fixed on screen; a
 *      ResizeObserver (rAF-coalesced, window-resize fallback) recomputes
 *      fit-to-width and re-clamps zoom/scroll when the dock is resized or
 *      returns from full-view.
 *
 * After every committed zoom AND pan (rAF-coalesced) it calls
 * `CTPS.timeline.rebuildRuler(pxPerSec, viewT0, viewT1)` so the ruler repaints
 * only the ticks near the visible window (bounded node count on long traces).
 *
 * ADR-053 carve-out: this file writes geometry ONLY through
 * `el.style.setProperty(...)` — the `--ctps-content-w` / `--ctps-tl-thumb-w` /
 * `--ctps-tl-thumb-x` custom properties — never `el.style.<prop> = ...`.
 * `viewport.scrollLeft = x` is a DOM property (the scroll position), not an
 * inline style, so it is allowed; `.value`, `.textContent`, `.disabled` and
 * `classList` are the only other state writes. Degenerate traces
 * (`duration_s <= 0` or `zoomMax <= 1`) add `ctps-tl--nozoom`, disable the
 * controls and hide the thumb, and never throw — the CSS fallbacks
 * (`var(--ctps-content-w, 100%)`) degrade the dock to a full-width,
 * non-zooming timeline if this module ever fails to hydrate, matching boot.js's
 * "one bad module can't take down the page" posture.
 *
 * Concatenated into its own <script> element AFTER timeline.js (so
 * `CTPS.timeline.rebuildRuler` and the built lane/ruler rows already exist) and
 * BEFORE boot.js (which runs the `init` this file pushes onto `CTPS._inits`).
 * Vanilla JS only — no frameworks, no network, works offline.
 */

"use strict";

(function () {
    var CTPS = window.CTPS;
    if (!CTPS || !CTPS._inits) {
        // player.js failed to create the namespace before this file ran —
        // nothing to register against and nowhere to record an error.
        return;
    }

    // ----- tunable constants ------------------------------------------------

    // Shortest window (seconds) that the maximum zoom lets fill the visible
    // track, and the absolute hard cap on the zoom factor (binds first on very
    // long traces — see the plan's Zoom/pan math).
    var MIN_VISIBLE_SECONDS = 1.5;
    var ABSOLUTE_MAX = 200;
    // Multiplicative step per button press / keypress.
    var ZOOM_STEP = 1.5;
    // Full slider travel (matches the range input's max in the markup).
    var SLIDER_MAX = 1000;
    // Wheel normalization: DOM_DELTA_LINE reports ~1 unit/line, scaled to px.
    var LINE_HEIGHT_PX = 16;
    // Ctrl/Cmd+wheel zoom sensitivity: exp(-deltaPx * this) per wheel event.
    var WHEEL_ZOOM_PER_PX = 0.0015;
    // Follow: page when the playhead comes within this many px of the right
    // edge (or scrolls behind the sticky gutter); re-seat it this fraction in
    // from the visible track's left edge.
    var EDGE_MARGIN = 24;
    var FOLLOW_ANCHOR_FRAC = 1 / 3;
    var EPS = 1e-6;

    // ----- module state (assigned in init) ---------------------------------

    var timelineEl = null; // .ctps-timeline (owns --ctps-content-w + nozoom)
    var viewport = null; // #ctps-tl-viewport (the horizontal scroll region)
    var content = null; // #ctps-tl-content (the scrolled, content-width child)
    var thumb = null; // #ctps-tl-thumb (custom scrollbar thumb)
    var scrollTrack = null; // .ctps-tl-scroll (thumb's containing track)
    var zoomSlider = null; // #ctps-zoom (range input)
    var zoomInBtn = null; // #ctps-zoom-in
    var zoomOutBtn = null; // #ctps-zoom-out
    var fitBtn = null; // #ctps-zoom-fit
    var levelReadout = null; // #ctps-zoom-level

    var store = null;
    var dur = 0; // track.duration_s
    var laneOffset = 0; // gutter + gap, in px (the track's left inset)

    var trackView = 0; // visible track width right of the sticky gutter (px)
    var fitContentW = 1; // content width at z = 1 (Math.floor of trackView)
    var zoomMax = 1; // capped maximum zoom factor
    var z = 1; // current zoom factor, always in [1, zoomMax]
    var contentW = 1; // current content track width (px) = fitContentW * z
    var nozoom = false; // degenerate/short trace — controls disabled

    var followSuspended = false; // a manual pan turned auto-follow off
    var _prevPlaying = false; // for the playing false->true re-arm edge
    var _selfScroll = false; // this module initiated the pending scroll write
    var _lastProgrammaticScrollLeft = 0; // post-clamp position of the last
    // programmatic scroll/width write; a browser clamp scroll lands on it
    var _rulerQueued = false; // an rAF ruler rebuild is already scheduled
    var _resizeQueued = false; // an rAF resize recompute is already scheduled

    // ----- small math helpers ----------------------------------------------

    /** Clamp `v` into [0, 1]. */
    function clamp01(v) {
        if (v < 0) {
            return 0;
        }
        if (v > 1) {
            return 1;
        }
        return v;
    }

    /** Clamp a zoom factor into the valid [1, zoomMax] range. */
    function clampZoom(value) {
        if (!(value > 1)) {
            return 1;
        }
        if (value > zoomMax) {
            return zoomMax;
        }
        return value;
    }

    // ----- metric compute (init + resize) ----------------------------------

    /**
     * Resolve the track's left inset `gutter + gap` (px). Prefers the
     * `.ctps-timeline` computed custom properties, but `--ctps-lane-gap` is
     * authored as `var(--ctps-space-3)` and some engines return that token
     * unresolved from `getComputedStyle`; in that case fall back to measuring
     * the stable ruler track's left offset within `#ctps-tl-content` (built by
     * timeline.js, whose init has already run, and `scrollLeft` is 0 at init),
     * then to the documented 52px gutter + 8px gap.
     */
    function readLaneOffset() {
        var cs = window.getComputedStyle(timelineEl);
        var gutter = parseFloat(cs.getPropertyValue("--ctps-lane-gutter"));
        var gap = parseFloat(cs.getPropertyValue("--ctps-lane-gap"));
        if (isFinite(gutter) && isFinite(gap)) {
            return gutter + gap;
        }
        var trackEl = document.getElementById("ctps-ruler-track") ||
            content.querySelector(".ctps-lane");
        if (trackEl) {
            var off = trackEl.getBoundingClientRect().left -
                content.getBoundingClientRect().left;
            if (isFinite(off) && off > 0) {
                return off;
            }
        }
        return (isFinite(gutter) ? gutter : 52) + (isFinite(gap) ? gap : 8);
    }

    /** Maximum zoom factor for the current duration (viewport-independent). */
    function computeZoomMax() {
        if (dur <= 0) {
            return 1;
        }
        var zm = dur / MIN_VISIBLE_SECONDS;
        if (zm < 1) {
            zm = 1;
        } else if (zm > ABSOLUTE_MAX) {
            zm = ABSOLUTE_MAX;
        }
        return zm;
    }

    /**
     * Recompute the fit-to-width metrics from the current viewport width.
     * `Math.floor` (per AC1) so a sub-pixel rounding surplus never shows a
     * thumb at 1x. Zoom max depends only on duration and is set once at init.
     */
    function computeFit() {
        trackView = viewport.clientWidth - laneOffset;
        fitContentW = Math.max(1, Math.floor(trackView));
    }

    // ----- content width + scroll writes (the ADR-053 carve-out) -----------

    /** Write the content track width (px) to `--ctps-content-w` on the dock. */
    function setContentW(widthPx) {
        contentW = widthPx;
        timelineEl.style.setProperty("--ctps-content-w", widthPx + "px");
        // A shrink can make the browser clamp scrollLeft during the forced
        // reflow and queue a native `scroll` event that no `_selfScroll` write
        // wraps. Reading scrollLeft flushes that layout; record the post-clamp
        // position so `onScroll` classifies the clamp event as self, not a pan.
        _lastProgrammaticScrollLeft = viewport.scrollLeft;
    }

    /** Max scrollLeft for the current (post-reflow) content width. */
    function maxScroll() {
        var m = viewport.scrollWidth - viewport.clientWidth;
        return m > 0 ? m : 0;
    }

    /** Clamp a scrollLeft into [0, maxScroll]. */
    function clampScroll(x) {
        var m = maxScroll();
        if (x < 0) {
            return 0;
        }
        if (x > m) {
            return m;
        }
        return x;
    }

    /** True when the content overflows the viewport (there is room to pan). */
    function canScroll() {
        return maxScroll() > 1;
    }

    /**
     * Programmatically set `scrollLeft`, clamped. Flags the write as
     * self-initiated so the `scroll` handler does not mistake the module's own
     * paging (follow, zoom re-anchor) for a manual pan. When the value would
     * not change, no `scroll` event fires — sync the thumb directly and leave
     * the guard clear so a later genuine scroll is still detected.
     */
    function setScrollLeft(x) {
        x = clampScroll(x);
        if (Math.abs(viewport.scrollLeft - x) < 0.5) {
            // No scroll event fires, but this IS the module's intended
            // position (it may equal a value a width-shrink already clamped
            // to); record it so the pending clamp `scroll` reads as self.
            _lastProgrammaticScrollLeft = viewport.scrollLeft;
            syncThumb();
            return;
        }
        _selfScroll = true;
        viewport.scrollLeft = x;
        _lastProgrammaticScrollLeft = viewport.scrollLeft;
    }

    // ----- zoom anchoring ---------------------------------------------------

    /**
     * Playhead-anchored new scrollLeft (buttons / slider / keys): keep the
     * playhead at the same on-screen x across the zoom step (AC4).
     */
    function playheadAnchor(oldContentW, newContentW, oldScrollLeft) {
        var frac = clamp01(store && dur > 0 ? store.t / dur : 0);
        var oldPlayheadX = laneOffset + frac * oldContentW;
        return (laneOffset + frac * newContentW) - (oldPlayheadX - oldScrollLeft);
    }

    /**
     * Cursor-anchored new scrollLeft (`Ctrl`/`Cmd`+wheel): keep the trace point
     * under the cursor fixed across the zoom step (AC3). Returns a closure over
     * the wheel event so it fits the same `resolveScroll` shape.
     */
    function cursorAnchor(evt) {
        return function (oldContentW, newContentW, oldScrollLeft) {
            var rect = viewport.getBoundingClientRect();
            var pointerX = evt.clientX - rect.left;
            var cursorContentX = oldScrollLeft + pointerX;
            var cursorFrac = clamp01((cursorContentX - laneOffset) / oldContentW);
            return (laneOffset + cursorFrac * newContentW) - pointerX;
        };
    }

    /**
     * Commit a zoom to `newZ` (clamped), re-anchoring the scroll via
     * `resolveScroll(oldContentW, newContentW, oldScrollLeft)`, then resync the
     * widget + thumb and schedule a windowed ruler rebuild. No-op when the
     * clamped factor is unchanged (already at fit or max).
     */
    function applyZoom(newZ, resolveScroll) {
        newZ = clampZoom(newZ);
        if (Math.abs(newZ - z) < EPS) {
            return;
        }
        var oldContentW = contentW;
        var oldScrollLeft = viewport.scrollLeft;
        z = newZ;
        setContentW(fitContentW * z);
        setScrollLeft(resolveScroll(oldContentW, contentW, oldScrollLeft));
        syncControls();
        scheduleRulerSync();
    }

    /** One multiplicative zoom step, playhead-anchored (no-op when nozoom). */
    function zoomStep(factor) {
        if (nozoom) {
            return;
        }
        applyZoom(z * factor, playheadAnchor);
    }

    /** Reset to fit-to-width, scroll to the start, and re-arm follow (AC5). */
    function fit() {
        followSuspended = false;
        if (nozoom) {
            setScrollLeft(0);
            return;
        }
        z = 1;
        setContentW(fitContentW);
        setScrollLeft(0);
        syncControls();
        scheduleRulerSync();
    }

    // ----- widget <-> zoom sync --------------------------------------------

    /** Slider position [0..SLIDER_MAX] -> zoom factor on a log scale. */
    function sliderToZoom(v) {
        if (zoomMax <= 1) {
            return 1;
        }
        return Math.pow(zoomMax, v / SLIDER_MAX);
    }

    /** Zoom factor -> slider position (inverse of sliderToZoom, rounded). */
    function zoomToSlider(factor) {
        if (zoomMax <= 1) {
            return 0;
        }
        return Math.round(SLIDER_MAX * Math.log(factor) / Math.log(zoomMax));
    }

    /** Push the current zoom onto the slider, readout, buttons, and thumb. */
    function syncControls() {
        if (zoomSlider) {
            zoomSlider.value = String(zoomToSlider(z));
        }
        if (levelReadout) {
            levelReadout.textContent = z.toFixed(1) + "×";
        }
        var atFit = z <= 1 + EPS;
        var atMax = z >= zoomMax - EPS;
        if (zoomOutBtn) {
            zoomOutBtn.disabled = atFit;
        }
        if (fitBtn) {
            fitBtn.disabled = atFit;
        }
        if (zoomInBtn) {
            zoomInBtn.disabled = atMax;
        }
        syncThumb();
    }

    /**
     * Size + place the custom scrollbar thumb from the live scroll geometry:
     * width = clientWidth / scrollWidth, left = scrollLeft / scrollWidth (both
     * percentages of the scroll track). Hidden (0% width) when degenerate or
     * when the content fits, so no thumb shows at 1x (AC1) or on a short trace.
     */
    function syncThumb() {
        if (!thumb) {
            return;
        }
        var scrollW = viewport.scrollWidth;
        var clientW = viewport.clientWidth;
        if (nozoom || scrollW <= 0 || (scrollW - clientW) <= 1) {
            thumb.style.setProperty("--ctps-tl-thumb-w", "0%");
            thumb.style.setProperty("--ctps-tl-thumb-x", "0%");
            return;
        }
        var wRatio = clientW / scrollW;
        var xRatio = viewport.scrollLeft / scrollW;
        thumb.style.setProperty("--ctps-tl-thumb-w", (wRatio * 100) + "%");
        thumb.style.setProperty("--ctps-tl-thumb-x", (xRatio * 100) + "%");
    }

    // ----- ruler coupling (rAF-coalesced) ----------------------------------

    /** Queue one windowed ruler rebuild on the next frame (coalesced). */
    function scheduleRulerSync() {
        if (_rulerQueued) {
            return;
        }
        _rulerQueued = true;
        window.requestAnimationFrame(function () {
            _rulerQueued = false;
            rulerSync();
        });
    }

    /**
     * Repaint the ruler ticks for the current visible window. `pxPerSec` is the
     * sole density input; the window `[viewT0, viewT1]` is derived from the
     * scroll offset (timeline.js widens it by its own one-screen margin).
     */
    function rulerSync() {
        if (!(CTPS.timeline && CTPS.timeline.rebuildRuler) || dur <= 0) {
            return;
        }
        var pxPerSec = contentW / dur;
        if (!(pxPerSec > 0)) {
            return;
        }
        var viewT0 = viewport.scrollLeft / pxPerSec;
        var viewT1 = viewT0 + (trackView > 0 ? trackView : viewport.clientWidth) / pxPerSec;
        try {
            CTPS.timeline.rebuildRuler(pxPerSec, viewT0, viewT1);
        } catch (rulerError) {
            CTPS._errors.push(
                "timelineView: rebuildRuler threw: " +
                (rulerError && rulerError.message || String(rulerError)));
        }
    }

    // ----- pan: wheel + thumb drag -----------------------------------------

    /** Normalize a wheel delta to px, honoring LINE / PAGE delta modes. */
    function normalizeDelta(evt, raw) {
        if (evt.deltaMode === 1) {
            return raw * LINE_HEIGHT_PX;
        }
        if (evt.deltaMode === 2) {
            return raw * viewport.clientWidth;
        }
        return raw;
    }

    /**
     * Single wheel handler (registered `{ passive: false }`):
     *   - `Ctrl`/`Cmd`+wheel -> cursor-anchored zoom + `preventDefault` (so the
     *     browser never page-zooms alongside).
     *   - else `|deltaY| > |deltaX|` -> horizontal pan by the normalized
     *     `deltaY` + `preventDefault` (only when there is room to pan).
     *   - else -> let the native `deltaX` (trackpad horizontal) scroll through.
     */
    function onWheel(evt) {
        if (nozoom) {
            return;
        }
        if (evt.ctrlKey || evt.metaKey) {
            evt.preventDefault();
            var deltaPx = normalizeDelta(evt, evt.deltaY);
            var newZ = clampZoom(z * Math.exp(-deltaPx * WHEEL_ZOOM_PER_PX));
            if (Math.abs(newZ - z) > EPS) {
                applyZoom(newZ, cursorAnchor(evt));
            }
            return;
        }
        if (Math.abs(evt.deltaY) > Math.abs(evt.deltaX)) {
            if (!canScroll()) {
                return;
            }
            evt.preventDefault();
            followSuspended = true;
            setScrollLeft(viewport.scrollLeft + normalizeDelta(evt, evt.deltaY));
        }
        // else: native deltaX horizontal scroll passes through untouched.
    }

    /**
     * Drag the custom thumb to pan. A thumb pixel move maps to a scroll move by
     * the scrollWidth/trackWidth ratio (the thumb width already encodes the
     * inverse), so the ruler + lanes + playhead track the drag exactly (AC7).
     * Pointer capture keeps move events flowing outside the thumb's bounds.
     */
    function wireThumbDrag() {
        if (!thumb) {
            return;
        }
        var dragging = false;
        var startX = 0;
        var startScroll = 0;

        thumb.addEventListener("pointerdown", function (evt) {
            if (nozoom || !canScroll()) {
                return;
            }
            dragging = true;
            startX = evt.clientX;
            startScroll = viewport.scrollLeft;
            followSuspended = true;
            if (thumb.setPointerCapture) {
                try {
                    thumb.setPointerCapture(evt.pointerId);
                } catch (captureError) {
                    // Best-effort; the drag still works via plain pointermove.
                }
            }
            evt.preventDefault();
        });

        thumb.addEventListener("pointermove", function (evt) {
            if (!dragging) {
                return;
            }
            var scrollW = viewport.scrollWidth;
            var trackW = scrollTrack ? scrollTrack.clientWidth : viewport.clientWidth;
            var deltaPx = evt.clientX - startX;
            var deltaScroll = (trackW > 0) ? deltaPx * (scrollW / trackW) : deltaPx;
            setScrollLeft(startScroll + deltaScroll);
        });

        function stopDrag() {
            dragging = false;
        }
        thumb.addEventListener("pointerup", stopDrag);
        thumb.addEventListener("pointercancel", stopDrag);
    }

    /**
     * On any `scroll`, keep the thumb + ruler in sync. A scroll that this
     * module did NOT initiate (native trackpad `deltaX`, keyboard) counts as a
     * manual pan and suspends follow while playing; a self-initiated scroll
     * (follow paging, zoom re-anchor) clears the guard and never suspends.
     *
     * The `_selfScroll` flag only covers a `scroll` that a `setScrollLeft`
     * write triggered. A width shrink (`setContentW` via zoom-out / `fit()`)
     * can make the browser clamp `scrollLeft` and fire a native `scroll` that
     * no such write wraps — its position lands on `_lastProgrammaticScrollLeft`
     * (recorded post-clamp). Treat that as self too, so it never falsely
     * suspends follow; only a scroll that moved AWAY from the last programmatic
     * position is a genuine user pan.
     */
    function onScroll() {
        syncThumb();
        scheduleRulerSync();
        if (_selfScroll) {
            _selfScroll = false;
            return;
        }
        if (Math.abs(viewport.scrollLeft - _lastProgrammaticScrollLeft) < 1) {
            return;
        }
        if (store && store.playing) {
            followSuspended = true;
        }
    }

    // ----- follow-the-playhead ---------------------------------------------

    /**
     * While playing and not suspended, page the view when the playhead nears
     * the right edge or scrolls behind the sticky gutter, re-seating it
     * ~FOLLOW_ANCHOR_FRAC in from the visible track's left edge (AC9). Uses the
     * self-scroll guard so this paging is never mistaken for a manual pan.
     */
    function followPlayhead() {
        if (nozoom || !store || !store.playing || followSuspended || !canScroll()) {
            return;
        }
        var frac = clamp01(dur > 0 ? store.t / dur : 0);
        var playheadContentX = laneOffset + frac * contentW;
        var playheadViewportX = playheadContentX - viewport.scrollLeft;
        var rightEdge = viewport.clientWidth - EDGE_MARGIN;
        if (playheadViewportX > rightEdge || playheadViewportX < laneOffset) {
            var targetViewportX =
                laneOffset + (viewport.clientWidth - laneOffset) * FOLLOW_ANCHOR_FRAC;
            setScrollLeft(playheadContentX - targetViewportX);
        }
    }

    /**
     * React to store `["t", "playing"]`: re-arm follow on the play false->true
     * edge (a fresh run may auto-scroll again), then follow the playhead on `t`.
     */
    function onStoreChange(changed) {
        if (changed.indexOf("playing") !== -1) {
            if (store.playing && !_prevPlaying) {
                followSuspended = false;
            }
            _prevPlaying = !!store.playing;
        }
        if (changed.indexOf("t") !== -1) {
            followPlayhead();
        }
    }

    // ----- degenerate / nozoom ---------------------------------------------

    /**
     * Degenerate guard (AC12): a zero/short trace or a viewport that already
     * fits at 1x cannot zoom. Fill the track at fit width, tag `.ctps-timeline`
     * with `ctps-tl--nozoom`, disable the controls, and hide the thumb — never
     * throw.
     */
    function applyNozoom() {
        nozoom = true;
        z = 1;
        setContentW(fitContentW);
        timelineEl.classList.add("ctps-tl--nozoom");
        if (zoomSlider) {
            zoomSlider.value = "0";
            zoomSlider.disabled = true;
        }
        if (zoomInBtn) {
            zoomInBtn.disabled = true;
        }
        if (zoomOutBtn) {
            zoomOutBtn.disabled = true;
        }
        if (fitBtn) {
            fitBtn.disabled = true;
        }
        if (levelReadout) {
            levelReadout.textContent = "1.0×";
        }
        syncThumb();
    }

    // ----- resize (mirror renderer.js: rAF-coalesced RO + window fallback) --

    /**
     * Recompute fit-to-width and re-apply the current zoom (clamped to the
     * unchanged zoomMax), re-ANCHOR the scroll so the playhead holds its
     * on-screen x across the width change, and resync the widget + ruler
     * (AC15). On a degenerate dock, just re-fill the track at the new width.
     */
    function recompute() {
        // Capture the pre-reflow geometry before computeFit rescales fit width
        // and before setContentW's reflow re-clamps scrollLeft, so the anchor
        // maps the OLD content-space playhead x to its new position.
        var oldContentW = contentW;
        var oldScrollLeft = viewport.scrollLeft;
        computeFit();
        if (nozoom) {
            setContentW(fitContentW);
            setScrollLeft(0);
            syncThumb();
            scheduleRulerSync();
            return;
        }
        z = clampZoom(z);
        var newContentW = fitContentW * z;
        setContentW(newContentW);
        setScrollLeft(playheadAnchor(oldContentW, newContentW, oldScrollLeft));
        syncControls();
        scheduleRulerSync();
    }

    /** Queue one resize recompute on the next frame (coalesced). */
    function scheduleResize() {
        if (_resizeQueued) {
            return;
        }
        _resizeQueued = true;
        window.requestAnimationFrame(function () {
            _resizeQueued = false;
            recompute();
        });
    }

    // ----- wiring -----------------------------------------------------------

    /** Bind the zoom widget controls. */
    function wireControls() {
        if (zoomInBtn) {
            zoomInBtn.addEventListener("click", function () {
                zoomStep(ZOOM_STEP);
            });
        }
        if (zoomOutBtn) {
            zoomOutBtn.addEventListener("click", function () {
                zoomStep(1 / ZOOM_STEP);
            });
        }
        if (fitBtn) {
            fitBtn.addEventListener("click", fit);
        }
        if (zoomSlider) {
            zoomSlider.addEventListener("input", function () {
                if (nozoom) {
                    return;
                }
                applyZoom(sliderToZoom(Number(zoomSlider.value)), playheadAnchor);
            });
        }
    }

    // ----- init (run by boot in load order) --------------------------------

    /**
     * Query the DOM contract, compute the fit/zoom metrics, expose
     * `CTPS.timelineView`, wire the widget/wheel/thumb/scroll/store/resize
     * seams, and paint the initial (fit-to-width) state. The whole body is
     * try/caught into `CTPS._errors`, matching every sibling module.
     */
    function init(ctx) {
        timelineEl = document.querySelector(".ctps-timeline");
        viewport = document.getElementById("ctps-tl-viewport");
        content = document.getElementById("ctps-tl-content");
        thumb = document.getElementById("ctps-tl-thumb");
        scrollTrack = document.querySelector(".ctps-tl-scroll");
        zoomSlider = document.getElementById("ctps-zoom");
        zoomInBtn = document.getElementById("ctps-zoom-in");
        zoomOutBtn = document.getElementById("ctps-zoom-out");
        fitBtn = document.getElementById("ctps-zoom-fit");
        levelReadout = document.getElementById("ctps-zoom-level");

        if (!timelineEl || !viewport || !content) {
            CTPS._errors.push("timelineView: timeline dock not found in DOM");
            return;
        }

        try {
            store = (ctx && ctx.store) || CTPS.store;
            var track = (ctx && ctx.track) ||
                (CTPS.engine && CTPS.engine.track) || null;
            dur = (track && track.duration_s) || 0;
            _prevPlaying = !!(store && store.playing);

            laneOffset = readLaneOffset();
            zoomMax = computeZoomMax();
            computeFit();
            nozoom = (dur <= 0) || (zoomMax <= 1);

            // Expose the transport keymap seam before any early-out below, so a
            // degenerate dock still answers zoomIn/zoomOut/fit as safe no-ops.
            CTPS.timelineView = {
                zoomIn: function () {
                    zoomStep(ZOOM_STEP);
                },
                zoomOut: function () {
                    zoomStep(1 / ZOOM_STEP);
                },
                fit: fit,
            };

            if (nozoom) {
                applyNozoom();
            } else {
                z = 1;
                setContentW(fitContentW);
                syncControls();
            }

            wireControls();
            viewport.addEventListener("wheel", onWheel, { passive: false });
            viewport.addEventListener("scroll", onScroll);
            wireThumbDrag();

            if (store) {
                store.subscribe(function (changedKeys) {
                    onStoreChange(changedKeys);
                });
            }

            // Mirror renderer.js: a ResizeObserver on the viewport (first
            // layout settle, window resize, full-view return) coalesced onto
            // one rAF, with a window-resize listener as the fallback.
            if (typeof ResizeObserver === "function") {
                new ResizeObserver(scheduleResize).observe(viewport);
            }
            window.addEventListener("resize", scheduleResize);

            scheduleRulerSync();
        } catch (initError) {
            CTPS._errors.push(
                "timelineView: init() threw: " +
                (initError && initError.message || String(initError)));
        }
    }

    CTPS._inits.push(init);
})();
