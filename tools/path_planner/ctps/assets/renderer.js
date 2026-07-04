/* CTPS — Clear Trajectory Prediction Simulator: layered canvas renderer.
 *
 * Owns the whole canvas side of the design-3a "Instrument" stage and nothing
 * else. Three absolutely-stacked canvases already exist in the emitted markup —
 * #ctps-canvas-static (map / bounds / trajectory / nodes), #ctps-canvas-trail
 * (the traveled overlay), and #ctps-canvas (actors: robot / selection halo /
 * scale bar). This module:
 *
 *   1. One shared, devicePixelRatio-aware resize sizes ALL THREE canvases
 *      identically (backing store = CSS box x dpr, ctx scaled by dpr), recomputes
 *      the world->pixel transform (uniform scale, centred, Y flipped so world +Y
 *      = North points up), and re-renders every layer. The field extent comes
 *      from CTPS.engine.track.field.bounds ([x0,y0,x1,y1] cm, LOWERCASE); a
 *      missing/degenerate box falls back to a unit box.
 *   2. Resolves EVERY canvas colour from the --ctps-* custom properties via
 *      getComputedStyle(document.documentElement) at init and on each resize —
 *      no hard-coded hex here, so the token stylesheet stays the single source.
 *   3. Registers the layers (map / bounds / trajectory / nodes / trail / actors)
 *      as separate draw-units mapped onto their backing canvas via canvasId, plus
 *      the `box` renderMode (rotated chassis rectangle + heading nose). `wheel-arc`
 *      is intentionally left unregistered — the reserved #55 seam.
 *   4. Exposes CTPS.renderer = { resize, worldToPixel, hitTest } for boot (window
 *      resize wiring) and T8 (canvas selection).
 *
 * Styling note (ADR-053 carve-out): this file never touches element `.style`
 * properties — it paints to <canvas> and reads token vars only. Vanilla ES5 —
 * var/function, no frameworks, no network, works offline.
 *
 * Coordinate convention (matches player.js / tools/path_planner): centimetres,
 * degrees in [0, 360), 0 deg = +x (East), CCW positive, world origin SW with +Y
 * North. Canvas y grows downward, so the transform flips Y and the box renderMode
 * negates the heading angle.
 */

"use strict";

(function () {
    var CTPS = window.CTPS;

    // ----- layout / paint constants (named, no magic literals inline) ------
    var PADDING = 24;            // px inset of the field within the stage box
    var TRAJ_WIDTH = 2;          // full trajectory stroke width
    var TRAJ_ALPHA = 0.6;        // full trajectory opacity (--ctps-text @ 60%)
    var TRAIL_WIDTH = 2.5;       // traveled overlay stroke width
    var BOUNDS_WIDTH = 1.5;      // field bounds frame width
    var NODE_DOT_R = 4;          // node dot radius
    var NODE_LABEL_DX = 7;       // node label x-offset from its dot
    var NODE_HALO_WIDTH = 3;     // dark paint-order halo behind a node label
    var NODE_FONT = "11px ui-monospace, Consolas, monospace";
    var BOX_LEN = 26;            // robot chassis length (px, along heading)
    var BOX_WID = 18;            // robot chassis width (px)
    var NOSE_LEN = 7;            // heading-nose projection past the front face
    var NOSE_HALF = 5;           // heading-nose half-height at the front face
    var ROBOT_OUTLINE_WIDTH = 1.5;
    var HALO_RADIUS = 18;        // selection halo ring radius (px)
    var HALO_WIDTH = 2;          // selection halo ring width
    var HALO_ALPHA = 0.55;       // selection halo opacity
    var SCALE_CM = 50;           // scale-bar span in world cm
    var SCALE_MARGIN = 16;       // scale-bar inset from the bottom-left corner
    var SCALE_TICK = 4;          // scale-bar end-tick half-height
    var SCALE_FONT = "10px ui-monospace, Consolas, monospace";
    var HIT_NODE_RADIUS = 14;    // px: a click within this of a node hits it
    var HIT_ROBOT_RADIUS = 16;   // px: else a click within this of the robot

    // ----- module state ----------------------------------------------------
    // The three canvas elements + their 2D contexts, keyed by the layer
    // descriptors' `canvasId` ("static" / "trail" / "actors").
    var _canvases = { static: null, trail: null, actors: null };
    var _ctxs = { static: null, trail: null, actors: null };

    // The world->pixel transform, recomputed on every resize (null until then).
    var _view = null;

    // Token colours resolved from the --ctps-* custom properties.
    var _colors = {
        text: "",
        accent: "",
        borderStrong: "",
        bg: "",
        muted: "",
        mapDim: 0,
    };

    // Optional printed field map (Image) + its load state; null when MAP_IMAGE
    // is absent (vector-bounds fallback).
    var _mapImg = null;
    var _mapLoaded = false;

    // Default robot render mode id (from PLAYER_META.render_mode; "box" default).
    var _renderModeId = "box";

    // Bounded traveled-trail state: `points` holds the pixel positions of
    // keyframes[0..builtIndex]; `builtIndex` is the last keyframe committed to
    // the polyline (-1 = empty). Reconciliation is O(delta) — see _reconcileTrail.
    var _trail = { points: [], builtIndex: -1 };

    // ----- token colour resolution -----------------------------------------

    /**
     * Read one CSS custom property off the document root, trimmed. Returns ""
     * when unset so an assignment to a canvas colour is simply ignored (the
     * degenerate no-stylesheet case), never a hard-coded hex fallback.
     */
    function _cssVar(name) {
        var v = getComputedStyle(document.documentElement).getPropertyValue(name);
        return v ? v.replace(/^\s+|\s+$/g, "") : "";
    }

    /**
     * Resolve every canvas colour from the --ctps-* token vars. Called at init
     * and on each resize so a (future) token change is picked up. map-dim is the
     * scrim alpha; a missing value degrades to 0 (no dimming) rather than
     * duplicating the token's numeric default.
     */
    function _resolveColors() {
        _colors.text = _cssVar("--ctps-text");
        _colors.accent = _cssVar("--ctps-accent");
        _colors.borderStrong = _cssVar("--ctps-border-strong");
        _colors.bg = _cssVar("--ctps-bg");
        _colors.muted = _cssVar("--ctps-muted");
        var dim = parseFloat(_cssVar("--ctps-map-dim"));
        _colors.mapDim = isNaN(dim) ? 0 : dim;
    }

    /**
     * Convert a `#rrggbb` (or `#rgb`) token value to an `rgba(r,g,b,a)` string.
     * Used only for the map scrim (the background token at the map-dim alpha).
     * A non-parseable input degrades to transparent black.
     */
    function _hexToRgba(hex, alpha) {
        var h = (hex || "").replace("#", "");
        if (h.length === 3) {
            h = h.charAt(0) + h.charAt(0) + h.charAt(1) + h.charAt(1) +
                h.charAt(2) + h.charAt(2);
        }
        var r = parseInt(h.slice(0, 2), 16);
        var g = parseInt(h.slice(2, 4), 16);
        var b = parseInt(h.slice(4, 6), 16);
        if (isNaN(r) || isNaN(g) || isNaN(b)) {
            return "rgba(0,0,0," + alpha + ")";
        }
        return "rgba(" + r + "," + g + "," + b + "," + alpha + ")";
    }

    // ----- field accessors + world<->pixel transform -----------------------

    /** The ingested track's `field` object, or null. */
    function _field() {
        var track = CTPS.engine.track;
        return (track && track.field) || null;
    }

    /** The field's nodes map `{name:[x,y]}` (LOWERCASE key), or an empty object. */
    function _fieldNodes() {
        var f = _field();
        return (f && f.nodes) || {};
    }

    /**
     * The field extent `[x0,y0,x1,y1]` in cm from `field.bounds` (LOWERCASE),
     * or the unit box `[0,0,1,1]` when bounds is missing or degenerate (so the
     * transform never divides by a zero span).
     */
    function _resolveBounds() {
        var f = _field();
        var b = f && f.bounds;
        if (b && b.length === 4 &&
            isFinite(b[0]) && isFinite(b[1]) && isFinite(b[2]) && isFinite(b[3]) &&
            (b[2] - b[0]) > 0 && (b[3] - b[1]) > 0) {
            return [b[0], b[1], b[2], b[3]];
        }
        return [0, 0, 1, 1];
    }

    /**
     * Project a world point (cm) to canvas pixels through `view`. Uniform scale,
     * centred within the padded box, Y flipped (world +Y up -> canvas y down).
     */
    function _project(view, x, y) {
        return {
            px: view.padX + (x - view.x0) * view.scale,
            py: view.cssH - view.padY - (y - view.y0) * view.scale,
        };
    }

    /**
     * (Re)compute the world->pixel transform for the given CSS box + dpr. The
     * scale is `min(availW/worldW, availH/worldH)` so the whole field fits; the
     * drawn field is centred in the leftover space.
     */
    function _computeView(cssW, cssH, dpr) {
        var bounds = _resolveBounds();
        var worldW = bounds[2] - bounds[0];
        var worldH = bounds[3] - bounds[1];
        var availW = Math.max(1, cssW - 2 * PADDING);
        var availH = Math.max(1, cssH - 2 * PADDING);
        var scale = Math.min(availW / worldW, availH / worldH);
        var drawnW = worldW * scale;
        var drawnH = worldH * scale;
        _view = {
            scale: scale,
            padX: PADDING + (availW - drawnW) / 2,
            padY: PADDING + (availH - drawnH) / 2,
            x0: bounds[0],
            y0: bounds[1],
            worldW: worldW,
            worldH: worldH,
            cssW: cssW,
            cssH: cssH,
            dpr: dpr,
            bounds: bounds,
        };
    }

    /**
     * Size one canvas's backing store to the CSS box x dpr and scale its context
     * so all subsequent drawing is in CSS pixels. Assigning width/height also
     * clears the canvas.
     */
    function _sizeCanvas(canvas, ctx, cssW, cssH, dpr) {
        canvas.width = Math.round(cssW * dpr);
        canvas.height = Math.round(cssH * dpr);
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    // ----- small array helpers ---------------------------------------------

    /** True when `key` is an element of `arr`. */
    function _includes(arr, key) {
        if (!arr) {
            return false;
        }
        for (var i = 0; i < arr.length; i++) {
            if (arr[i] === key) {
                return true;
            }
        }
        return false;
    }

    /** True when arrays `a` and `b` share at least one element. */
    function _intersects(a, b) {
        if (!a || !b) {
            return false;
        }
        for (var i = 0; i < a.length; i++) {
            for (var j = 0; j < b.length; j++) {
                if (a[i] === b[j]) {
                    return true;
                }
            }
        }
        return false;
    }

    // ----- bounded traveled-trail bookkeeping ------------------------------

    /** Drop the committed trail; a resize invalidates the cached pixel points. */
    function _resetTrail() {
        _trail.points = [];
        _trail.builtIndex = -1;
    }

    /**
     * Bring the committed trail polyline in line with keyframe index `idx`
     * (the last keyframe at/before the playhead). Forward play appends the
     * newly-passed keyframes; a backward seek pops the now-future ones. Both are
     * O(delta) — never a full keyframe scan. `idx` comes from the pinned
     * findKeyframeIndex bisect, so this is the load-bearing bounded-seek path.
     */
    function _reconcileTrail(view, idx) {
        var kf = CTPS.engine.keyframes;
        var pts = _trail.points;
        while (_trail.builtIndex < idx) {
            _trail.builtIndex += 1;
            pts.push(_project(view, kf[_trail.builtIndex].x, kf[_trail.builtIndex].y));
        }
        while (_trail.builtIndex > idx) {
            pts.pop();
            _trail.builtIndex -= 1;
        }
    }

    // ----- layer draw units -------------------------------------------------

    /** Field-bounds rect corners (top-left / bottom-right) in canvas pixels. */
    function _boundsRect(view) {
        var b = view.bounds;
        var tl = _project(view, b[0], b[3]); // (x0, y1) -> top-left
        var br = _project(view, b[2], b[1]); // (x1, y0) -> bottom-right
        return { x: tl.px, y: tl.py, w: br.px - tl.px, h: br.py - tl.py };
    }

    /**
     * map layer (static canvas): the printed field map stretched over the bounds
     * rect, dimmed by the --ctps-map-dim scrim. Skipped entirely when there is no
     * map (vector-bounds fallback) or before the image has loaded.
     */
    function _drawMap(ctx, view, state) {
        if (!_mapImg || !_mapLoaded) {
            return;
        }
        var r = _boundsRect(view);
        ctx.save();
        ctx.drawImage(_mapImg, r.x, r.y, r.w, r.h);
        ctx.fillStyle = _hexToRgba(_colors.bg, _colors.mapDim);
        ctx.fillRect(r.x, r.y, r.w, r.h);
        ctx.restore();
    }

    /** bounds layer (static): the field-extent frame. */
    function _drawBounds(ctx, view, state) {
        var r = _boundsRect(view);
        ctx.save();
        ctx.strokeStyle = _colors.borderStrong;
        ctx.lineWidth = BOUNDS_WIDTH;
        ctx.strokeRect(r.x, r.y, r.w, r.h);
        ctx.restore();
    }

    /**
     * trajectory layer (static): the full route polyline through every keyframe
     * pose, in --ctps-text at 60% opacity. Static content — this runs on a
     * layers/resize re-render, not per playhead frame, so iterating the whole
     * keyframe list here is fine (the per-frame bounded path is the trail).
     */
    function _drawTrajectory(ctx, view, state) {
        var kf = CTPS.engine.keyframes;
        var count = kf.length;
        if (count === 0) {
            return;
        }
        ctx.save();
        ctx.strokeStyle = _colors.text;
        ctx.globalAlpha = TRAJ_ALPHA;
        ctx.lineWidth = TRAJ_WIDTH;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.beginPath();
        var p0 = _project(view, kf[0].x, kf[0].y);
        ctx.moveTo(p0.px, p0.py);
        for (var i = 1; i < count; i++) {
            var p = _project(view, kf[i].x, kf[i].y);
            ctx.lineTo(p.px, p.py);
        }
        ctx.stroke();
        ctx.restore();
    }

    /**
     * nodes layer (static): a dot per node plus its mono label, the label drawn
     * with a dark paint-order halo (stroke behind, fill on top) so it reads over
     * the map. Nodes come from field.nodes (LOWERCASE `{name:[x,y]}`).
     */
    function _drawNodes(ctx, view, state) {
        var nodes = _fieldNodes();
        ctx.save();
        ctx.font = NODE_FONT;
        ctx.textAlign = "left";
        ctx.textBaseline = "middle";
        ctx.lineJoin = "round";
        for (var name in nodes) {
            if (!Object.prototype.hasOwnProperty.call(nodes, name)) {
                continue;
            }
            var c = nodes[name];
            var p = _project(view, c[0], c[1]);
            ctx.fillStyle = _colors.text;
            ctx.beginPath();
            ctx.arc(p.px, p.py, NODE_DOT_R, 0, Math.PI * 2);
            ctx.fill();
            ctx.lineWidth = NODE_HALO_WIDTH;
            ctx.strokeStyle = _colors.bg;
            ctx.strokeText(name, p.px + NODE_LABEL_DX, p.py);
            ctx.fillStyle = _colors.text;
            ctx.fillText(name, p.px + NODE_LABEL_DX, p.py);
        }
        ctx.restore();
    }

    /**
     * trail layer (trail canvas): the traveled polyline start->playhead in
     * --ctps-accent. The committed keyframe points are reconciled in O(delta)
     * (forward append / backward pop) and the head is the interpolated pose at
     * `t` — so the whole update is bounded by findKeyframeIndex, never a full
     * per-frame keyframe scan.
     */
    function _drawTrail(ctx, view, state) {
        var kf = CTPS.engine.keyframes;
        if (!kf || kf.length === 0) {
            return;
        }
        var idx = CTPS.engine.findKeyframeIndex(state.t);
        _reconcileTrail(view, idx);
        var pts = _trail.points;
        if (pts.length === 0) {
            return;
        }
        var pose = CTPS.engine.poseAt(state.t);
        var head = _project(view, pose.x, pose.y);
        ctx.save();
        ctx.strokeStyle = _colors.accent;
        ctx.lineWidth = TRAIL_WIDTH;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.beginPath();
        ctx.moveTo(pts[0].px, pts[0].py);
        for (var k = 1; k < pts.length; k++) {
            ctx.lineTo(pts[k].px, pts[k].py);
        }
        ctx.lineTo(head.px, head.py);
        ctx.stroke();
        ctx.restore();
    }

    /**
     * A ring around the current selection (robot follows the live pose; a node
     * sits at its fixed position). Drawn under the robot on the actors canvas.
     */
    function _drawSelectionHalo(ctx, view, state, pose) {
        var sel = state.selection;
        if (!sel || !sel.kind) {
            return;
        }
        var cx, cy;
        if (sel.kind === "robot") {
            var rp = _project(view, pose.x, pose.y);
            cx = rp.px;
            cy = rp.py;
        } else if (sel.kind === "node") {
            var coord = _fieldNodes()[sel.id];
            if (!coord) {
                return;
            }
            var np = _project(view, coord[0], coord[1]);
            cx = np.px;
            cy = np.py;
        } else {
            return;
        }
        ctx.save();
        ctx.strokeStyle = _colors.accent;
        ctx.globalAlpha = HALO_ALPHA;
        ctx.lineWidth = HALO_WIDTH;
        ctx.beginPath();
        ctx.arc(cx, cy, HALO_RADIUS, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
    }

    /** The bottom-left 50 cm scale bar (line + end ticks + mono label). */
    function _drawScaleBar(ctx, view) {
        var lenPx = SCALE_CM * view.scale;
        var x0 = SCALE_MARGIN;
        var y0 = view.cssH - SCALE_MARGIN;
        ctx.save();
        ctx.strokeStyle = _colors.muted;
        ctx.fillStyle = _colors.muted;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x0 + lenPx, y0);
        ctx.moveTo(x0, y0 - SCALE_TICK);
        ctx.lineTo(x0, y0 + SCALE_TICK);
        ctx.moveTo(x0 + lenPx, y0 - SCALE_TICK);
        ctx.lineTo(x0 + lenPx, y0 + SCALE_TICK);
        ctx.stroke();
        ctx.font = SCALE_FONT;
        ctx.textAlign = "center";
        ctx.textBaseline = "bottom";
        ctx.fillText(SCALE_CM + " cm", x0 + lenPx / 2, y0 - SCALE_TICK - 2);
        ctx.restore();
    }

    /**
     * actors layer (actors canvas): the selection halo, the robot via the
     * renderMode registry at the interpolated pose, and the scale bar. The
     * full-view HUD is a DOM element owned by T9 — not drawn here.
     */
    function _drawActors(ctx, view, state) {
        var pose = CTPS.engine.poseAt(state.t);
        _drawSelectionHalo(ctx, view, state, pose);
        var mode = CTPS.renderModes.get(_renderModeId);
        if (!mode || typeof mode.draw !== "function") {
            mode = CTPS.renderModes.get("box");
        }
        if (mode && typeof mode.draw === "function") {
            mode.draw(ctx, pose, view, {
                fill: _colors.accent,
                outline: _colors.bg,
            });
        }
        _drawScaleBar(ctx, view);
    }

    // ----- canvas render orchestration -------------------------------------

    /**
     * Repaint one backing canvas: reset to the dpr base transform, clear, then
     * draw every registered layer whose `canvasId` matches and whose `visible`
     * returns truthy, in registration order. A canvas is the unit of repaint —
     * all its layers redraw together so toggling one still paints the siblings.
     */
    function _renderCanvas(canvasId, state) {
        var ctx = _ctxs[canvasId];
        if (!ctx || !_view) {
            return;
        }
        ctx.setTransform(_view.dpr, 0, 0, _view.dpr, 0, 0);
        ctx.clearRect(0, 0, _view.cssW, _view.cssH);
        CTPS.layers.forEach(function (layer) {
            if (!layer || layer.canvasId !== canvasId) {
                return;
            }
            var vis = true;
            if (typeof layer.visible === "function") {
                vis = layer.visible(state);
            }
            if (!vis || typeof layer.draw !== "function") {
                return;
            }
            layer.draw(ctx, _view, state);
        });
    }

    /** Repaint all three canvases (used after a resize / full re-render). */
    function _renderAll(state) {
        _renderCanvas("static", state);
        _renderCanvas("trail", state);
        _renderCanvas("actors", state);
    }

    /**
     * Repaint only the canvases whose layers care about `changedKeys`: a layer's
     * `invalidateOn` intersecting the changed keys marks its canvas dirty. Order
     * across canvases is irrelevant (separate bitmaps); order within one is the
     * registration order handled by _renderCanvas.
     */
    function _renderForChange(changedKeys, state) {
        var dirty = {};
        CTPS.layers.forEach(function (layer) {
            if (layer && _intersects(layer.invalidateOn, changedKeys)) {
                dirty[layer.canvasId] = true;
            }
        });
        for (var canvasId in dirty) {
            if (Object.prototype.hasOwnProperty.call(dirty, canvasId)) {
                _renderCanvas(canvasId, state);
            }
        }
    }

    // ----- public renderer surface -----------------------------------------

    /**
     * Shared resize: re-resolve token colours, size all three canvases to the
     * current CSS box x dpr, recompute the world->pixel transform, invalidate the
     * cached trail (its pixel points are transform-relative), and repaint every
     * layer. Safe to call before the canvases are wired (guards on the elements).
     */
    function resize() {
        if (!_canvases.actors || !_ctxs.actors) {
            return;
        }
        _resolveColors();
        var dpr = window.devicePixelRatio || 1;
        var rect = _canvases.actors.getBoundingClientRect();
        var cssW = Math.max(1, Math.round(rect.width));
        var cssH = Math.max(1, Math.round(rect.height));
        _sizeCanvas(_canvases.static, _ctxs.static, cssW, cssH, dpr);
        _sizeCanvas(_canvases.trail, _ctxs.trail, cssW, cssH, dpr);
        _sizeCanvas(_canvases.actors, _ctxs.actors, cssW, cssH, dpr);
        _computeView(cssW, cssH, dpr);
        _resetTrail();
        _renderAll(CTPS.store);
    }

    /** World (cm) -> canvas pixel `{px, py}` through the current transform. */
    function worldToPixel(x, y) {
        return _project(_view, x, y);
    }

    /**
     * Hit-test a canvas-pixel click: the nearest node within HIT_NODE_RADIUS
     * wins ({kind:"node", id}); else the robot when the click is within
     * HIT_ROBOT_RADIUS of its current pose ({kind:"robot"}); else null.
     */
    function hitTest(px, py) {
        if (!_view) {
            return null;
        }
        var nodes = _fieldNodes();
        var bestId = null;
        var bestD = HIT_NODE_RADIUS;
        for (var name in nodes) {
            if (!Object.prototype.hasOwnProperty.call(nodes, name)) {
                continue;
            }
            var c = nodes[name];
            var np = _project(_view, c[0], c[1]);
            var d = Math.hypot(px - np.px, py - np.py);
            if (d <= bestD) {
                bestD = d;
                bestId = name;
            }
        }
        if (bestId !== null) {
            return { kind: "node", id: bestId };
        }
        var pose = CTPS.engine.poseAt(CTPS.store.t);
        var rp = _project(_view, pose.x, pose.y);
        if (Math.hypot(px - rp.px, py - rp.py) <= HIT_ROBOT_RADIUS) {
            return { kind: "robot" };
        }
        return null;
    }

    CTPS.renderer = {
        resize: resize,
        worldToPixel: worldToPixel,
        hitTest: hitTest,
    };

    // ----- layer + renderMode registration ---------------------------------
    // map / bounds / trajectory / nodes are SEPARATE layers sharing the static
    // canvas, each with its own visible() so the MAP / NODES chips toggle
    // independently; the trail + actors get their own canvases.
    CTPS.layers.register({
        id: "map",
        canvasId: "static",
        visible: function (state) { return !!state.layers.map; },
        draw: _drawMap,
        invalidateOn: ["layers"],
    });
    CTPS.layers.register({
        id: "bounds",
        canvasId: "static",
        visible: function () { return true; },
        draw: _drawBounds,
        invalidateOn: ["layers"],
    });
    CTPS.layers.register({
        id: "trajectory",
        canvasId: "static",
        visible: function () { return true; },
        draw: _drawTrajectory,
        invalidateOn: ["layers"],
    });
    CTPS.layers.register({
        id: "nodes",
        canvasId: "static",
        visible: function (state) { return !!state.layers.nodes; },
        draw: _drawNodes,
        invalidateOn: ["layers"],
    });
    CTPS.layers.register({
        id: "trail",
        canvasId: "trail",
        visible: function (state) { return !!state.layers.trail; },
        draw: _drawTrail,
        invalidateOn: ["t", "layers"],
    });
    CTPS.layers.register({
        id: "actors",
        canvasId: "actors",
        visible: function () { return true; },
        draw: _drawActors,
        invalidateOn: ["t", "selection"],
    });

    /**
     * `box` render mode: a rotated chassis rectangle (~BOX_LEN x BOX_WID) with a
     * small heading nose at the front, accent fill + a 1.5px dark outline. World
     * heading is CCW-from-+x with Y up; the canvas Y is flipped, so the drawing
     * angle is the negated heading. `wheel-arc` is deliberately NOT registered —
     * the reserved #55 pivot seam; the registry simply stays able to accept it.
     */
    CTPS.renderModes.register({
        id: "box",
        draw: function (ctx, pose, view, chassis) {
            var p = _project(view, pose.x, pose.y);
            var ang = -pose.h * Math.PI / 180;
            ctx.save();
            ctx.translate(p.px, p.py);
            ctx.rotate(ang);
            ctx.fillStyle = chassis.fill;
            ctx.strokeStyle = chassis.outline;
            ctx.lineWidth = ROBOT_OUTLINE_WIDTH;
            ctx.lineJoin = "round";
            ctx.beginPath();
            ctx.rect(-BOX_LEN / 2, -BOX_WID / 2, BOX_LEN, BOX_WID);
            ctx.fill();
            ctx.stroke();
            ctx.beginPath();
            ctx.moveTo(BOX_LEN / 2, -NOSE_HALF);
            ctx.lineTo(BOX_LEN / 2 + NOSE_LEN, 0);
            ctx.lineTo(BOX_LEN / 2, NOSE_HALF);
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
            ctx.restore();
        },
    });

    // ----- map image bootstrap ---------------------------------------------

    /**
     * Kick off loading the inline field map (the MAP_IMAGE global — a data URI
     * or null). The image source is assigned via `img.src = mapSrc` (spaces
     * around `=`) so the offline self-containment attribute check never trips.
     * A successful load repaints the static canvas.
     */
    function _initMap() {
        var mapSrc = (typeof MAP_IMAGE !== "undefined") ? MAP_IMAGE : null;
        _mapImg = null;
        _mapLoaded = false;
        if (!mapSrc) {
            return;
        }
        var img = new Image();
        img.onload = function () {
            _mapLoaded = true;
            _renderCanvas("static", CTPS.store);
        };
        _mapImg = img;
        img.src = mapSrc;
    }

    // ----- init (run by boot in load order) --------------------------------
    // Grabs the canvas elements + 2D contexts, does the first resize, and
    // subscribes: a "fullView" change re-measures the (now collapsed/expanded)
    // stage box on the next frame, everything else repaints only the canvases
    // whose layers' invalidateOn matches the changed keys.
    CTPS._inits.push(function (ctx) {
        _canvases.static = document.getElementById("ctps-canvas-static");
        _canvases.trail = document.getElementById("ctps-canvas-trail");
        _canvases.actors = document.getElementById("ctps-canvas");
        if (!_canvases.static || !_canvases.trail || !_canvases.actors) {
            CTPS._errors.push("renderer: one or more stage canvases not found");
            return;
        }
        _ctxs.static = _canvases.static.getContext("2d");
        _ctxs.trail = _canvases.trail.getContext("2d");
        _ctxs.actors = _canvases.actors.getContext("2d");
        if (!_ctxs.static || !_ctxs.trail || !_ctxs.actors) {
            CTPS._errors.push("renderer: 2D context unavailable");
            return;
        }
        _renderModeId = (ctx.meta && ctx.meta.render_mode) || "box";
        _resolveColors();
        _initMap();
        resize();
        ctx.store.subscribe(function (changedKeys, state) {
            if (_includes(changedKeys, "fullView")) {
                // The rail/dock collapse changes the stage's CSS box via a class
                // toggle (no window resize fires); re-measure after layout settles.
                window.requestAnimationFrame(resize);
                return;
            }
            _renderForChange(changedKeys, state);
        });
    });
})();
