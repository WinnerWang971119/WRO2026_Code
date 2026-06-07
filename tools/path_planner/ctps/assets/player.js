/* CTPS — Clear Trajectory Prediction Simulator: self-contained Canvas player.
 *
 * Runtime for the HTML emitted by player_html.render(). Reads the inline
 * `TRACK` constant (Data Model B), draws the static field + trajectory "T"
 * once, then animates an oriented robot marker over it. The transport bar
 * (play/pause, seek slider, time readout) drives a playhead clock and the
 * side panel narrates the current sub-command.
 *
 * Vanilla JS only — no frameworks, no network, works on file://.
 *
 * Coordinate convention (matches tools/path_planner): centimetres, degrees in
 * [0, 360), 0 deg = +x (East), CCW positive. Field origin (0,0) is the SW
 * corner with Y pointing North/up, so the world->pixel transform flips Y to
 * canvas (Y-down) space while preserving aspect ratio.
 */

"use strict";

(function () {
    // ----- track + DOM handles -------------------------------------------
    var track = (typeof TRACK !== "undefined") ? TRACK : null;
    if (!track) {
        return;
    }

    var keyframes = Array.isArray(track.keyframes) ? track.keyframes : [];
    var events = Array.isArray(track.events) ? track.events : [];
    var field = track.field || {};
    var durationS = (typeof track.duration_s === "number") ? track.duration_s : 0;

    var canvas = document.getElementById("ctps-canvas");
    var ctx = canvas ? canvas.getContext("2d") : null;
    var playBtn = document.getElementById("ctps-play");
    var slider = document.getElementById("ctps-seek");
    var timeReadout = document.getElementById("ctps-time");
    var cmdLabel = document.getElementById("ctps-command");
    var logEl = document.getElementById("ctps-log");

    // ----- transform (world cm -> canvas px), aspect-preserving ----------
    var BOUNDS = normalizeBounds(field.bounds);
    var STATIC_LAYER = null; // ImageData snapshot of the static field render.
    // Optional printed field map (base64 data URI injected as the MAP_IMAGE
    // global). Loaded asynchronously; until it is ready the static layer falls
    // back to the vector field render so a map-less track still plays.
    var mapImage = null;
    var mapReady = false;
    var view = {
        scale: 1.0,
        offsetX: 0.0,
        offsetY: 0.0,
        pixelWidth: 0,
        pixelHeight: 0,
    };

    var PADDING_PX = 16;
    var NODE_RADIUS_PX = 4;
    var ROBOT_LEN_PX = 16;
    var ROBOT_WID_PX = 11;

    /**
     * Normalize a bounds array into {x0, y0, x1, y1} with a strictly positive
     * extent, falling back to a unit box so a degenerate field never produces
     * a divide-by-zero in the transform.
     */
    function normalizeBounds(b) {
        if (!Array.isArray(b) || b.length < 4) {
            return { x0: 0, y0: 0, x1: 1, y1: 1 };
        }
        var x0 = Math.min(b[0], b[2]);
        var y0 = Math.min(b[1], b[3]);
        var x1 = Math.max(b[0], b[2]);
        var y1 = Math.max(b[1], b[3]);
        if (x1 - x0 <= 0) { x1 = x0 + 1; }
        if (y1 - y0 <= 0) { y1 = y0 + 1; }
        return { x0: x0, y0: y0, x1: x1, y1: y1 };
    }

    /** World x (cm, East+) -> canvas x (px). */
    function worldToPixelX(xCm) {
        return view.offsetX + (xCm - BOUNDS.x0) * view.scale;
    }

    /** World y (cm, North+) -> canvas y (px). Flips Y so North is up. */
    function worldToPixelY(yCm) {
        return view.offsetY + (BOUNDS.y1 - yCm) * view.scale;
    }

    /**
     * Recompute the world->pixel transform from the current canvas size.
     * Uniform scale (min of the two axis ratios) preserves aspect ratio; the
     * scaled field is centred within the padded canvas.
     */
    function recomputeTransform() {
        var w = view.pixelWidth;
        var h = view.pixelHeight;
        var worldW = BOUNDS.x1 - BOUNDS.x0;
        var worldH = BOUNDS.y1 - BOUNDS.y0;
        var availW = Math.max(1, w - 2 * PADDING_PX);
        var availH = Math.max(1, h - 2 * PADDING_PX);
        var scale = Math.min(availW / worldW, availH / worldH);
        if (!isFinite(scale) || scale <= 0) {
            scale = 1.0;
        }
        view.scale = scale;
        view.offsetX = (w - worldW * scale) / 2.0;
        view.offsetY = (h - worldH * scale) / 2.0;
    }

    // =====================================================================
    // Seek — AC3a guarantee.
    //
    // findKeyframeIndex(t) is a BOUNDED binary search (bisect) over the baked
    // keyframe array: each step halves [lo, hi], so it runs in O(log N) and
    // touches at most ~log2(N) entries — it does NOT scan the full array. The
    // loop bound is the keyframe count, never the playhead's frame count.
    // poseAt(t) uses that index to lerp between the two bracketing keyframes
    // (x/y linearly, heading on the shortest arc). This is the load-bearing
    // O(1)-class seek path required by AC3a / TC12.
    // =====================================================================

    /**
     * Return the index `i` of the last keyframe whose time is <= `t`
     * (the left element of the bracketing pair). Bounded binary search:
     * the search window strictly shrinks each iteration, capped by the
     * keyframe count. Returns 0 for an empty/degenerate track or a `t`
     * before the first keyframe; returns the last index for a `t` at or
     * past the final keyframe.
     */
    function findKeyframeIndex(t) {
        var n = keyframes.length;
        if (n === 0) {
            return 0;
        }
        if (t <= keyframes[0].t) {
            return 0;
        }
        if (t >= keyframes[n - 1].t) {
            return n - 1;
        }
        var lo = 0;
        var hi = n - 1;
        // Invariant: keyframes[lo].t <= t < keyframes[hi].t. Each step halves
        // the window via Math.floor of the midpoint — bounded by log2(n).
        while (hi - lo > 1) {
            var mid = lo + Math.floor((hi - lo) / 2);
            if (keyframes[mid].t <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    /**
     * Wrap a heading delta (deg) into (-180, +180] so the marker rotates over
     * the shortest arc instead of spinning the long way round.
     */
    function wrapDeltaDeg(d) {
        var x = d % 360.0;
        if (x <= -180.0) {
            x += 360.0;
        } else if (x > 180.0) {
            x -= 360.0;
        }
        return x;
    }

    /**
     * Resolve the interpolated pose {x, y, h} at time `t`. Uses
     * findKeyframeIndex (bounded binary search) to bracket `t`, then lerps:
     * x/y linearly and heading on the shortest arc. Guards the lerp against
     * 0- or 1-keyframe tracks and against a zero-length time bracket.
     */
    function poseAt(t) {
        var n = keyframes.length;
        if (n === 0) {
            return { x: 0, y: 0, h: 0 };
        }
        if (n === 1) {
            return { x: keyframes[0].x, y: keyframes[0].y, h: keyframes[0].h };
        }
        var i = findKeyframeIndex(t);
        if (i >= n - 1) {
            return {
                x: keyframes[n - 1].x,
                y: keyframes[n - 1].y,
                h: keyframes[n - 1].h,
            };
        }
        var a = keyframes[i];
        var b = keyframes[i + 1];
        var span = b.t - a.t;
        var u = (span > 0) ? (t - a.t) / span : 0.0;
        if (u < 0) { u = 0; }
        if (u > 1) { u = 1; }
        return {
            x: a.x + (b.x - a.x) * u,
            y: a.y + (b.y - a.y) * u,
            h: a.h + wrapDeltaDeg(b.h - a.h) * u,
        };
    }

    // ----- static field render (drawn once, cached as ImageData) ----------

    /** Draw the bounds frame as a rectangle. */
    function drawBounds() {
        var px0 = worldToPixelX(BOUNDS.x0);
        var py0 = worldToPixelY(BOUNDS.y1); // top edge = max Y in canvas space
        var px1 = worldToPixelX(BOUNDS.x1);
        var py1 = worldToPixelY(BOUNDS.y0);
        ctx.strokeStyle = "#3a4253";
        ctx.lineWidth = 1.5;
        ctx.strokeRect(px0, py0, px1 - px0, py1 - py0);
    }

    /** Draw every obstacle rectangle (x0,y0,x1,y1,kind). */
    function drawObstacles() {
        var obstacles = Array.isArray(field.obstacles) ? field.obstacles : [];
        for (var i = 0; i < obstacles.length; i++) {
            var o = obstacles[i];
            if (!Array.isArray(o) || o.length < 4) {
                continue;
            }
            var ox0 = worldToPixelX(Math.min(o[0], o[2]));
            var oy0 = worldToPixelY(Math.max(o[1], o[3]));
            var ox1 = worldToPixelX(Math.max(o[0], o[2]));
            var oy1 = worldToPixelY(Math.min(o[1], o[3]));
            ctx.fillStyle = "rgba(214, 96, 84, 0.30)";
            ctx.fillRect(ox0, oy0, ox1 - ox0, oy1 - oy0);
            ctx.strokeStyle = "rgba(214, 96, 84, 0.85)";
            ctx.lineWidth = 1.0;
            ctx.strokeRect(ox0, oy0, ox1 - ox0, oy1 - oy0);
        }
    }

    /** Draw the trajectory "T" as a polyline through the keyframe poses. */
    function drawTrajectory() {
        if (keyframes.length < 2) {
            return;
        }
        ctx.strokeStyle = "#4f9dff";
        ctx.lineWidth = 2.0;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.beginPath();
        ctx.moveTo(worldToPixelX(keyframes[0].x), worldToPixelY(keyframes[0].y));
        for (var i = 1; i < keyframes.length; i++) {
            ctx.lineTo(
                worldToPixelX(keyframes[i].x),
                worldToPixelY(keyframes[i].y)
            );
        }
        ctx.stroke();
    }

    /** Draw every node as a dot plus its name label. */
    function drawNodes() {
        var nodes = field.nodes || {};
        ctx.font = "11px ui-monospace, monospace";
        ctx.textBaseline = "middle";
        for (var name in nodes) {
            if (!Object.prototype.hasOwnProperty.call(nodes, name)) {
                continue;
            }
            var p = nodes[name];
            if (!Array.isArray(p) || p.length < 2) {
                continue;
            }
            var cx = worldToPixelX(p[0]);
            var cy = worldToPixelY(p[1]);
            ctx.fillStyle = "#cdd6e0";
            ctx.beginPath();
            ctx.arc(cx, cy, NODE_RADIUS_PX, 0, Math.PI * 2);
            ctx.fill();
            ctx.fillStyle = "#8b97a8";
            ctx.fillText(name, cx + NODE_RADIUS_PX + 3, cy);
        }
    }

    /**
     * Draw the printed field map over the bounds rectangle. The map is authored
     * to span the whole field (matching the visualizer's
     * extent=(0, width_cm, 0, height_cm)), so it covers the bounds rect exactly;
     * worldToPixelY flips Y so the image top edge lands on the North (max-Y)
     * field edge.
     */
    function drawMap() {
        if (!mapReady || !mapImage) {
            return;
        }
        var px0 = worldToPixelX(BOUNDS.x0);
        var py0 = worldToPixelY(BOUNDS.y1); // top edge = max Y in canvas space
        var w = (BOUNDS.x1 - BOUNDS.x0) * view.scale;
        var h = (BOUNDS.y1 - BOUNDS.y0) * view.scale;
        ctx.drawImage(mapImage, px0, py0, w, h);
    }

    /**
     * Render the static layer and cache it as ImageData so per-frame redraws
     * only restore the background and stamp the moving robot marker on top.
     *
     * With a printed map loaded, the map shows the field and obstacles, so the
     * overlay is just the trajectory and node markers (robot is added per
     * frame). Without a map, fall back to the full vector render (bounds frame +
     * obstacle boxes) so a map-less track — LRS, fixtures, or a missing map
     * file — still draws a usable field.
     */
    function renderStaticLayer() {
        ctx.fillStyle = "#11151c";
        ctx.fillRect(0, 0, view.pixelWidth, view.pixelHeight);
        if (mapReady) {
            drawMap();
        } else {
            drawBounds();
            drawObstacles();
        }
        drawTrajectory();
        drawNodes();
        // Snapshot the FULL backing store in DEVICE pixels. getImageData and
        // putImageData ignore the canvas transform and address the backing
        // store directly, so the snapshot must use canvas.width/height (device
        // px), not view.pixelWidth/height (CSS px). On a HiDPI display
        // (devicePixelRatio > 1) a CSS-px snapshot covers only the top-left
        // 1/dpr of the canvas, leaving the rest un-restored — every scrubbed
        // robot arrow then persists as an afterimage. Device-px capture clears
        // the whole frame on each redraw.
        STATIC_LAYER = ctx.getImageData(0, 0, canvas.width, canvas.height);
    }

    /** Draw the oriented robot marker (an arrow triangle) at a pose. */
    function drawRobot(pose) {
        var cx = worldToPixelX(pose.x);
        var cy = worldToPixelY(pose.y);
        // World heading is CCW from +x with Y up; canvas Y is flipped, so a
        // CCW world rotation is a CW canvas rotation -> negate the angle.
        var rad = -pose.h * Math.PI / 180.0;
        var cos = Math.cos(rad);
        var sin = Math.sin(rad);
        var nose = { x: ROBOT_LEN_PX, y: 0 };
        var tailL = { x: -ROBOT_LEN_PX * 0.6, y: -ROBOT_WID_PX };
        var tailR = { x: -ROBOT_LEN_PX * 0.6, y: ROBOT_WID_PX };
        function tx(p) { return cx + p.x * cos - p.y * sin; }
        function ty(p) { return cy + p.x * sin + p.y * cos; }
        ctx.beginPath();
        ctx.moveTo(tx(nose), ty(nose));
        ctx.lineTo(tx(tailL), ty(tailL));
        ctx.lineTo(tx(tailR), ty(tailR));
        ctx.closePath();
        ctx.fillStyle = "#ffd24a";
        ctx.fill();
        ctx.strokeStyle = "#11151c";
        ctx.lineWidth = 1.5;
        ctx.stroke();
    }

    /**
     * Redraw the frame: restore the cached static field, then stamp the robot
     * marker for the pose at the current playhead time.
     */
    function drawFrame() {
        if (!STATIC_LAYER) {
            return;
        }
        ctx.putImageData(STATIC_LAYER, 0, 0);
        drawRobot(poseAt(playhead.t));
    }

    // ----- events / narration --------------------------------------------

    var WARN_TYPE = "drift_warn";

    /** Human-friendly prefix for an event type shown in the command line. */
    function eventPrefix(type) {
        switch (type) {
            case "task_start": return "Task";
            case "task_end": return "Task end";
            case "goto": return "goTo";
            case "manip": return "Manipulator";
            case "wait": return "wait";
            case "drift_warn": return "DRIFT";
            case "print": return "print";
            case "beep": return "beep";
            case "led": return "led";
            default: return type;
        }
    }

    /**
     * Index of the last event whose time is <= `t`, or -1 if none. Bounded
     * binary search over the (non-decreasing) event array — same bounded
     * pattern as the keyframe lookup.
     */
    function findEventIndex(t) {
        var n = events.length;
        if (n === 0 || t < events[0].t) {
            return -1;
        }
        if (t >= events[n - 1].t) {
            return n - 1;
        }
        var lo = 0;
        var hi = n - 1;
        while (hi - lo > 1) {
            var mid = lo + Math.floor((hi - lo) / 2);
            if (events[mid].t <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    var lastLoggedEventIndex = -1;

    /** Format one event as a side-panel line. */
    function formatEvent(ev) {
        return eventPrefix(ev.type) + ": " + (ev.label || "");
    }

    /**
     * Update the side panel for the current playhead time: set the current
     * command line (most recent event at/before the playhead) and append any
     * newly-crossed events to the scrolling log.
     */
    function updateNarration() {
        var idx = findEventIndex(playhead.t);
        if (cmdLabel) {
            if (idx < 0) {
                cmdLabel.textContent = "—";
                cmdLabel.className = "ctps-command";
            } else {
                var cur = events[idx];
                cmdLabel.textContent = formatEvent(cur);
                cmdLabel.className = (cur.type === WARN_TYPE)
                    ? "ctps-command ctps-warn"
                    : "ctps-command";
            }
        }
        if (!logEl) {
            lastLoggedEventIndex = idx;
            return;
        }
        // Crossing forward: append every newly-passed event. Seeking backward:
        // rebuild the log up to the current index so it always matches.
        if (idx < lastLoggedEventIndex) {
            rebuildLog(idx);
        } else if (idx > lastLoggedEventIndex) {
            for (var i = lastLoggedEventIndex + 1; i <= idx; i++) {
                appendLogLine(events[i]);
            }
        }
        lastLoggedEventIndex = idx;
    }

    /** Append one event as a log line (warning-styled for drift). */
    function appendLogLine(ev) {
        var line = document.createElement("div");
        line.className = (ev.type === WARN_TYPE)
            ? "ctps-log-line ctps-warn"
            : "ctps-log-line";
        line.textContent = "[" + ev.t.toFixed(2) + "s] " + formatEvent(ev);
        logEl.appendChild(line);
        logEl.scrollTop = logEl.scrollHeight;
    }

    /** Clear and rebuild the log so it contains events [0..upTo]. */
    function rebuildLog(upTo) {
        while (logEl.firstChild) {
            logEl.removeChild(logEl.firstChild);
        }
        for (var i = 0; i <= upTo; i++) {
            appendLogLine(events[i]);
        }
    }

    // ----- playback clock + transport ------------------------------------

    var SLIDER_MAX = 1000;
    var playhead = { t: 0.0 };
    var playing = false;
    var lastFrameMs = 0;

    /** Format seconds as a "t / duration" readout string. */
    function formatTime() {
        return playhead.t.toFixed(2) + " / " + durationS.toFixed(2) + " s";
    }

    /** Sync the slider position from the current playhead time. */
    function syncSliderFromPlayhead() {
        if (!slider) {
            return;
        }
        var frac = (durationS > 0) ? (playhead.t / durationS) : 0.0;
        slider.value = String(Math.round(frac * SLIDER_MAX));
    }

    /** Render readout + slider + narration + canvas for the current time. */
    function refreshAll() {
        if (timeReadout) {
            timeReadout.textContent = formatTime();
        }
        syncSliderFromPlayhead();
        updateNarration();
        drawFrame();
    }

    /**
     * Seek the playhead to an absolute time (clamped to [0, duration]). This
     * is the bounded-search seek path: poseAt() resolves the pose via
     * findKeyframeIndex (binary search), never a full scan.
     */
    function seekTo(t) {
        if (t < 0) { t = 0; }
        if (t > durationS) { t = durationS; }
        playhead.t = t;
        refreshAll();
    }

    /** Toggle play/pause; resets the frame-delta baseline when resuming. */
    function setPlaying(on) {
        playing = on;
        if (playBtn) {
            playBtn.textContent = on ? "Pause" : "Play";
        }
        if (on) {
            // Restart from the beginning if parked at the end.
            if (playhead.t >= durationS) {
                playhead.t = 0;
            }
            lastFrameMs = 0;
            requestAnimationFrame(tick);
        }
    }

    /**
     * requestAnimationFrame loop. Advances the playhead by real elapsed time
     * while playing, stops at the end, and redraws each frame. During play we
     * advance an O(1) incremental clock; the pose is still resolved through
     * the bounded poseAt() path.
     */
    function tick(nowMs) {
        if (!playing) {
            return;
        }
        if (lastFrameMs === 0) {
            lastFrameMs = nowMs;
        }
        var dt = (nowMs - lastFrameMs) / 1000.0;
        lastFrameMs = nowMs;
        playhead.t += dt;
        if (playhead.t >= durationS) {
            playhead.t = durationS;
            refreshAll();
            setPlaying(false);
            return;
        }
        refreshAll();
        requestAnimationFrame(tick);
    }

    // ----- canvas sizing --------------------------------------------------

    /**
     * Size the canvas backing store to its CSS box (accounting for device
     * pixel ratio), recompute the transform, re-render the static layer, and
     * redraw. Called on init and on window resize.
     */
    function resizeCanvas() {
        if (!canvas || !ctx) {
            return;
        }
        var dpr = window.devicePixelRatio || 1;
        var cssW = canvas.clientWidth || 640;
        var cssH = canvas.clientHeight || 360;
        canvas.width = Math.max(1, Math.round(cssW * dpr));
        canvas.height = Math.max(1, Math.round(cssH * dpr));
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        view.pixelWidth = cssW;
        view.pixelHeight = cssH;
        recomputeTransform();
        renderStaticLayer();
        drawFrame();
    }

    /**
     * Kick off loading the optional field-map background. Reads the inline
     * MAP_IMAGE global (a base64 data URI, or null when no map was embedded);
     * does nothing when absent. resizeCanvas() has already rendered the static
     * layer once (map-less) by the time the image's async onload fires, so the
     * handler re-renders the static layer with the map and redraws the frame.
     */
    function loadMapImage() {
        var src = (typeof MAP_IMAGE !== "undefined") ? MAP_IMAGE : null;
        if (!src) {
            return;
        }
        var img = new Image();
        img.onload = function () {
            mapImage = img;
            mapReady = true;
            renderStaticLayer();
            drawFrame();
        };
        img.src = src;
    }

    // ----- wiring ---------------------------------------------------------

    function init() {
        if (slider) {
            slider.min = "0";
            slider.max = String(SLIDER_MAX);
            slider.step = "1";
            slider.value = "0";
            slider.addEventListener("input", function () {
                var frac = Number(slider.value) / SLIDER_MAX;
                if (playing) {
                    setPlaying(false);
                }
                seekTo(frac * durationS);
            });
        }
        if (playBtn) {
            playBtn.textContent = "Play";
            playBtn.addEventListener("click", function () {
                setPlaying(!playing);
            });
        }
        window.addEventListener("resize", resizeCanvas);
        resizeCanvas();
        seekTo(0);
        loadMapImage();
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", init);
    } else {
        init();
    }
})();
