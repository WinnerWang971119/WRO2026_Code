/* CTPS — Layout Editor: core (view transform + field backdrop + tab state).
 *
 * Seeds the editor's own namespace (`window.CTPS_EDITOR`, distinct from the
 * player's `window.CTPS`) and owns the canvas side of the two-tab offline editor
 * (object/cargo plan, T8 -- issue #57):
 *
 *   - a DUPLICATED `computeView` (the ~40-line world->pixel transform, copied
 *     from renderer.js's private `_computeView` per the plan's Contracts ->
 *     Projection; renderer.js is byte-frozen and NOT given an export), plus the
 *     pixel->world `unproject` INVERSE (editor-only -- the player never drags),
 *   - the Field-tab field backdrop (map image or the vector bounds / obstacles /
 *     nodes) drawn from `const FIELD`,
 *   - the Robot-tab backdrop: the default robot box at `2 * robot_radius_cm` per
 *     side, plus the manipulator zones via `window.CTPS_OBJ.drawZones`,
 *   - object rendering via `window.CTPS_OBJ.drawObject`, the selection halo, the
 *     live rubber-band draft, and a fit-to-view scale bar,
 *   - Field/Robot tab state + the single clear+redraw repaint (never a
 *     getImageData/putImageData snapshot -- the device-px ghost trap).
 *
 * Vanilla ES5 (var/function), works offline. Styling touches DOM only through
 * the `hidden` property + classList (never an inline element style); all colour
 * lives on the <canvas> and is read from the tokens.css custom properties, the
 * same getComputedStyle channel renderer.js uses.
 */

"use strict";

(function () {
    var E = (window.CTPS_EDITOR = window.CTPS_EDITOR || {});
    var OBJ = window.CTPS_OBJ; // objects_draw.js (inlined first): project/drawObject/drawZones.

    // ----- layout / paint constants (named, no magic literals inline) ------
    var PADDING = 24;            // px inset of the drawn field within the stage box
    var ROBOT_FALLBACK_CM = 9;   // robot half-side (cm) when the field omits a radius
    var ROBOT_MARGIN_CM = 5;     // extra world margin around the Robot-tab contents
    var ROBOT_OUTLINE_W = 2;     // robot-box outline stroke width (px)
    var ROBOT_CORNER = 0.3;      // robot-box rounded-corner radius (frac of half-side)
    var ROBOT_ARROW_TIP = 0.28;  // heading-arrow tip x (frac of half-side)
    var ROBOT_ARROW_BACK = 0.18; // heading-arrow base setback (frac of half-side)
    var ROBOT_ARROW_HALF = 0.2;  // heading-arrow base half-width (frac of half-side)
    var NODE_DOT_R = 4;          // field-node dot radius (px)
    var NODE_LABEL_DX = 7;       // field-node label x-offset from its dot (px)
    var NODE_HALO_W = 3;         // dark paint-order halo behind a node label (px)
    var NODE_FONT = "11px ui-monospace, Consolas, monospace";
    var LABEL_FONT = "10px ui-monospace, Consolas, monospace";
    var BOUNDS_W = 1.5;          // field-bounds frame stroke width (px)
    var OBSTACLE_FILL = "rgba(214, 96, 84, 0.30)";   // translucent obstacle fill
    var OBSTACLE_STROKE = "rgba(214, 96, 84, 0.85)"; // stronger obstacle outline
    var OBSTACLE_W = 1;          // obstacle outline stroke width (px)
    var SEL_W = 2;               // selection-halo stroke width (px)
    var SEL_PAD = 3;             // selection-halo padding beyond the object (px)
    var SEL_DASH = [4, 3];       // selection-halo dash pattern (px)
    var DRAFT_W = 1.5;           // rubber-band draft stroke width (px)
    var DRAFT_DASH = [5, 4];     // rubber-band draft dash pattern (px)
    var DROP_DOT_R = 3;          // drop-point marker radius (px)
    var SCALE_MARGIN = 16;       // scale-bar inset from the bottom-left corner (px)
    var SCALE_TICK = 4;          // scale-bar end-tick half-height (px)
    var SCALE_FONT = "10px ui-monospace, Consolas, monospace";
    var SCALE_CANDIDATES = [100, 50, 20, 10, 5, 2, 1]; // nice scale-bar spans (cm)
    var SCALE_MAX_FRAC = 0.45;   // scale bar may occupy at most this frac of the width

    // ----- token colour resolution (cached; static in this app) ------------
    var _colors = null;

    /** Read one CSS custom property off the document root, trimmed ("" if unset). */
    function _cssVar(name) {
        var v = getComputedStyle(document.documentElement).getPropertyValue(name);
        return v ? v.replace(/^\s+|\s+$/g, "") : "";
    }

    /** Resolve + cache the canvas token colours from the inlined stylesheet. */
    function _resolveColors() {
        _colors = {
            text: _cssVar("--ctps-text"),
            accent: _cssVar("--ctps-accent"),
            muted: _cssVar("--ctps-muted"),
            borderStrong: _cssVar("--ctps-border-strong"),
            bg: _cssVar("--ctps-bg"),
            hold: _cssVar("--ctps-hold"),
            mapDim: parseFloat(_cssVar("--ctps-map-dim")),
        };
        if (isNaN(_colors.mapDim)) {
            _colors.mapDim = 0;
        }
    }

    /** The cached token table, resolving it on first use. */
    function _theColors() {
        if (!_colors) {
            _resolveColors();
        }
        return _colors;
    }

    /**
     * Convert a `#rrggbb` (or `#rgb`) token value to an `rgba(r,g,b,a)` string,
     * used only for the map scrim. A non-parseable input degrades to transparent
     * black. A verbatim duplicate of renderer.js's private `_hexToRgba`.
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

    // ----- field-backdrop data accessors -----------------------------------

    /** The inlined `const FIELD` block, or a degenerate empty field. */
    function _field() {
        if (typeof FIELD !== "undefined" && FIELD) {
            return FIELD;
        }
        return { nodes: {}, obstacles: [], bounds: [0, 0, 1, 1], robot_radius_cm: 0 };
    }

    /**
     * The field extent `[x0,y0,x1,y1]` in cm from `FIELD.bounds`, or the unit box
     * when bounds is missing/degenerate (so the transform never divides by zero).
     */
    function _fieldBounds() {
        var b = _field().bounds;
        if (b && b.length === 4 &&
            isFinite(b[0]) && isFinite(b[1]) && isFinite(b[2]) && isFinite(b[3]) &&
            (b[2] - b[0]) > 0 && (b[3] - b[1]) > 0) {
            return [b[0], b[1], b[2], b[3]];
        }
        return [0, 0, 1, 1];
    }

    /** The robot half-side in world cm: `robot_radius_cm`, or a fixed fallback. */
    function _robotHalfCm() {
        var r = _field().robot_radius_cm;
        return (typeof r === "number" && r > 0) ? r : ROBOT_FALLBACK_CM;
    }

    /**
     * The Robot-tab world extent `[x0,y0,x1,y1]` (robot-LOCAL cm): the union of
     * the robot box, both zones, and every drop point, plus a margin. The Robot
     * tab renders the local frame directly (pose identity), so local cm ARE the
     * view's world cm.
     */
    function _robotBounds() {
        var half = _robotHalfCm();
        var box = { minX: -half, minY: -half, maxX: half, maxY: half };
        var rob = E.state.robot;
        function ext(x, y) {
            if (x < box.minX) { box.minX = x; }
            if (x > box.maxX) { box.maxX = x; }
            if (y < box.minY) { box.minY = y; }
            if (y > box.maxY) { box.maxY = y; }
        }
        if (rob) {
            var zones = [rob.grab_zone, rob.box_zone];
            for (var i = 0; i < zones.length; i++) {
                var z = zones[i];
                if (z) {
                    ext(z.x_cm - z.w_cm / 2, z.y_cm - z.h_cm / 2);
                    ext(z.x_cm + z.w_cm / 2, z.y_cm + z.h_cm / 2);
                }
            }
            var drops = rob.drop_points || [];
            for (var j = 0; j < drops.length; j++) {
                if (drops[j]) {
                    ext(drops[j].x_cm, drops[j].y_cm);
                }
            }
        }
        return [
            box.minX - ROBOT_MARGIN_CM, box.minY - ROBOT_MARGIN_CM,
            box.maxX + ROBOT_MARGIN_CM, box.maxY + ROBOT_MARGIN_CM,
        ];
    }

    // ----- world<->pixel transform (DUPLICATE of renderer.js _computeView) --

    /**
     * (Re)compute the world->pixel transform for the given world `bounds` + CSS
     * box + dpr. Uniform scale so the whole extent fits, centred in the leftover
     * space, Y flipped (world +Y up -> canvas y down). A duplicate of
     * renderer.js's private `_computeView` (parameterised on `bounds` so both the
     * Field and Robot tabs reuse it) -- renderer.js is not refactored.
     *
     * @param bounds world extent `[x0,y0,x1,y1]` in cm.
     * @param cssW stage width in CSS px.
     * @param cssH stage height in CSS px.
     * @param dpr devicePixelRatio.
     * @return the computed `view` transform.
     */
    function computeView(bounds, cssW, cssH, dpr) {
        var worldW = bounds[2] - bounds[0];
        var worldH = bounds[3] - bounds[1];
        var availW = Math.max(1, cssW - 2 * PADDING);
        var availH = Math.max(1, cssH - 2 * PADDING);
        var scale = Math.min(availW / worldW, availH / worldH);
        var drawnW = worldW * scale;
        var drawnH = worldH * scale;
        return {
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
     * The pixel->world INVERSE of `OBJ.project` (editor-only). Given a CSS-pixel
     * point on the canvas, return its world (cm) coordinates through `view`.
     *
     * @param view a transform from `computeView`.
     * @param px canvas x in CSS px.
     * @param py canvas y in CSS px.
     * @return `{ x, y }` world coordinates in cm.
     */
    function unproject(view, px, py) {
        return {
            x: view.x0 + (px - view.padX) / view.scale,
            y: view.y0 + (view.cssH - view.padY - py) / view.scale,
        };
    }

    /** The transform for the currently active tab. */
    function activeView() {
        return E.state.activeTab === "robot" ? E.state.robotView : E.state.fieldView;
    }

    // ----- optional field-map background -----------------------------------
    var _mapImg = null;
    var _mapLoaded = false;

    /**
     * Kick off loading the inline field map (the `MAP_IMAGE` global -- a data URI
     * or null). The source is assigned via `img.src = mapSrc` (spaces around `=`)
     * so the self-containment attribute check never trips. A successful load
     * repaints.
     */
    function initMap() {
        var mapSrc = (typeof MAP_IMAGE !== "undefined") ? MAP_IMAGE : null;
        _mapImg = null;
        _mapLoaded = false;
        if (!mapSrc) {
            return;
        }
        var img = new Image();
        img.onload = function () {
            _mapLoaded = true;
            redraw();
        };
        _mapImg = img;
        img.src = mapSrc;
    }

    // ----- field-backdrop drawing (Field tab) ------------------------------

    /** Field-bounds rect (top-left / size) in canvas pixels. */
    function _boundsRect(view) {
        var b = view.bounds;
        var tl = OBJ.project(view, b[0], b[3]); // (x0, y1) -> top-left
        var br = OBJ.project(view, b[2], b[1]); // (x1, y0) -> bottom-right
        return { x: tl.px, y: tl.py, w: br.px - tl.px, h: br.py - tl.py };
    }

    /** Draw the printed map (if loaded + dimmed), else the vector obstacles. */
    function _drawFieldSurface(ctx, view, r) {
        var C = _theColors();
        if (_mapImg && _mapLoaded) {
            ctx.save();
            ctx.drawImage(_mapImg, r.x, r.y, r.w, r.h);
            ctx.fillStyle = _hexToRgba(C.bg, C.mapDim);
            ctx.fillRect(r.x, r.y, r.w, r.h);
            ctx.restore();
            return;
        }
        var obstacles = _field().obstacles || [];
        if (obstacles.length === 0) {
            return;
        }
        ctx.save();
        ctx.fillStyle = OBSTACLE_FILL;
        ctx.strokeStyle = OBSTACLE_STROKE;
        ctx.lineWidth = OBSTACLE_W;
        for (var i = 0; i < obstacles.length; i++) {
            var o = obstacles[i];
            if (!o || o.length < 4) {
                continue;
            }
            var tl = OBJ.project(view, Math.min(o[0], o[2]), Math.max(o[1], o[3]));
            var br = OBJ.project(view, Math.max(o[0], o[2]), Math.min(o[1], o[3]));
            ctx.fillRect(tl.px, tl.py, br.px - tl.px, br.py - tl.py);
            ctx.strokeRect(tl.px, tl.py, br.px - tl.px, br.py - tl.py);
        }
        ctx.restore();
    }

    /** Draw a dot + mono label per field node, the label over a dark halo. */
    function _drawNodes(ctx, view) {
        var C = _theColors();
        var nodes = _field().nodes || {};
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
            var p = OBJ.project(view, c[0], c[1]);
            ctx.fillStyle = C.text;
            ctx.beginPath();
            ctx.arc(p.px, p.py, NODE_DOT_R, 0, Math.PI * 2);
            ctx.fill();
            ctx.lineWidth = NODE_HALO_W;
            ctx.strokeStyle = C.bg;
            ctx.strokeText(name, p.px + NODE_LABEL_DX, p.py);
            ctx.fillStyle = C.text;
            ctx.fillText(name, p.px + NODE_LABEL_DX, p.py);
        }
        ctx.restore();
    }

    /** Draw the full Field-tab backdrop: surface + bounds frame + nodes. */
    function drawFieldBackdrop(ctx, view) {
        var C = _theColors();
        var r = _boundsRect(view);
        _drawFieldSurface(ctx, view, r);
        ctx.save();
        ctx.strokeStyle = C.borderStrong;
        ctx.lineWidth = BOUNDS_W;
        ctx.strokeRect(r.x, r.y, r.w, r.h);
        ctx.restore();
        _drawNodes(ctx, view);
    }

    /** Draw every authored field object through `OBJ.drawObject`. */
    function drawObjects(ctx, view) {
        var objs = E.state.objects;
        for (var i = 0; i < objs.length; i++) {
            OBJ.drawObject(ctx, view, objs[i], OBJ.project);
        }
    }

    // ----- robot-backdrop drawing (Robot tab) ------------------------------

    /** Trace a rounded rectangle (arcTo corners), radius clamped to half the side. */
    function _roundRectPath(ctx, x, y, w, h, rad) {
        rad = Math.min(rad, w / 2, h / 2);
        ctx.beginPath();
        ctx.moveTo(x + rad, y);
        ctx.arcTo(x + w, y, x + w, y + h, rad);
        ctx.arcTo(x + w, y + h, x, y + h, rad);
        ctx.arcTo(x, y + h, x, y, rad);
        ctx.arcTo(x, y, x + w, y, rad);
        ctx.closePath();
    }

    /**
     * Draw the default robot box centred at local (0,0), side `2 * robotHalfCm`,
     * with a heading arrow along +x (heading-forward). Hollow accent outline so
     * zones underneath stay visible.
     */
    function drawRobotBox(ctx, view) {
        var C = _theColors();
        var half = _robotHalfCm();
        var p = OBJ.project(view, 0, 0);
        var hp = half * view.scale; // pixel half-side (heading 0 -> no rotation)
        ctx.save();
        ctx.translate(p.px, p.py);
        ctx.strokeStyle = C.accent;
        ctx.lineWidth = ROBOT_OUTLINE_W;
        ctx.lineJoin = "round";
        _roundRectPath(ctx, -hp, -hp, hp * 2, hp * 2, hp * ROBOT_CORNER);
        ctx.stroke();
        ctx.fillStyle = C.accent;
        ctx.beginPath();
        ctx.moveTo(hp * ROBOT_ARROW_TIP, 0);
        ctx.lineTo(-hp * ROBOT_ARROW_BACK, -hp * ROBOT_ARROW_HALF);
        ctx.lineTo(-hp * ROBOT_ARROW_BACK, hp * ROBOT_ARROW_HALF);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    }

    /** Label the grab zone, box zone, and each drop point in muted text. */
    function _drawZoneLabels(ctx, view, rob) {
        var C = _theColors();
        ctx.save();
        ctx.font = LABEL_FONT;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillStyle = C.muted;
        if (rob.grab_zone) {
            var g = OBJ.project(view, rob.grab_zone.x_cm, rob.grab_zone.y_cm);
            ctx.fillText("GRAB", g.px, g.py);
        }
        if (rob.box_zone) {
            var b = OBJ.project(view, rob.box_zone.x_cm, rob.box_zone.y_cm);
            ctx.fillText("BOX", b.px, b.py);
        }
        var drops = rob.drop_points || [];
        ctx.textAlign = "left";
        for (var i = 0; i < drops.length; i++) {
            if (!drops[i]) {
                continue;
            }
            var p = OBJ.project(view, drops[i].x_cm, drops[i].y_cm);
            ctx.fillText(String(i), p.px + 7, p.py - 6);
        }
        ctx.restore();
    }

    /** Draw the Robot-tab backdrop: robot box + zones overlay + labels. */
    function drawRobotBackdrop(ctx, view) {
        drawRobotBox(ctx, view);
        var rob = E.state.robot;
        if (rob) {
            // Pose identity -> the zones' robot-local cm map straight to world cm.
            OBJ.drawZones(ctx, view, { x: 0, y: 0, h: 0 }, rob, OBJ.project);
            _drawZoneLabels(ctx, view, rob);
        }
    }

    // ----- selection halo + rubber-band draft ------------------------------

    /** Stroke a robot-local (pose-identity) rect as a projected polygon. */
    function _strokeLocalRect(ctx, view, z) {
        var hw = z.w_cm / 2;
        var hh = z.h_cm / 2;
        var corners = [
            [z.x_cm - hw, z.y_cm - hh], [z.x_cm + hw, z.y_cm - hh],
            [z.x_cm + hw, z.y_cm + hh], [z.x_cm - hw, z.y_cm + hh],
        ];
        ctx.beginPath();
        for (var i = 0; i < corners.length; i++) {
            var p = OBJ.project(view, corners[i][0], corners[i][1]);
            if (i === 0) {
                ctx.moveTo(p.px, p.py);
            } else {
                ctx.lineTo(p.px, p.py);
            }
        }
        ctx.closePath();
        ctx.stroke();
    }

    /** Draw the accent halo around whatever is currently selected. */
    function drawSelection(ctx, view) {
        var s = E.state.selection;
        if (!s) {
            return;
        }
        var C = _theColors();
        ctx.save();
        ctx.strokeStyle = C.accent;
        ctx.lineWidth = SEL_W;
        if (ctx.setLineDash) {
            ctx.setLineDash(SEL_DASH);
        }
        if (s.kind === "object") {
            var o = objectById(s.id);
            if (o) {
                var c = OBJ.project(view, o.x_cm, o.y_cm);
                if (o.shape === "circle") {
                    var rr = (o.radius_cm || 0) * view.scale + SEL_PAD;
                    ctx.beginPath();
                    ctx.arc(c.px, c.py, rr, 0, Math.PI * 2);
                    ctx.stroke();
                } else {
                    var wp = (o.w_cm || 0) * view.scale + 2 * SEL_PAD;
                    var hpx = (o.h_cm || 0) * view.scale + 2 * SEL_PAD;
                    var ang = -(o.heading_deg || 0) * Math.PI / 180;
                    ctx.translate(c.px, c.py);
                    ctx.rotate(ang);
                    ctx.strokeRect(-wp / 2, -hpx / 2, wp, hpx);
                }
            }
        } else if (s.kind === "grab_zone" || s.kind === "box_zone") {
            var z = E.state.robot ? E.state.robot[s.kind] : null;
            if (z) {
                _strokeLocalRect(ctx, view, z);
            }
        } else if (s.kind === "drop") {
            var dp = (E.state.robot && E.state.robot.drop_points)
                ? E.state.robot.drop_points[s.index] : null;
            if (dp) {
                var pp = OBJ.project(view, dp.x_cm, dp.y_cm);
                ctx.beginPath();
                ctx.arc(pp.px, pp.py, DROP_DOT_R + SEL_PAD, 0, Math.PI * 2);
                ctx.stroke();
            }
        }
        ctx.restore();
    }

    /** Draw the live rubber-band preview while a rect/circle is being dragged. */
    function drawDraft(ctx, view) {
        var d = E.state.draft;
        if (!d) {
            return;
        }
        var C = _theColors();
        ctx.save();
        ctx.strokeStyle = C.accent;
        ctx.lineWidth = DRAFT_W;
        if (ctx.setLineDash) {
            ctx.setLineDash(DRAFT_DASH);
        }
        var p0 = OBJ.project(view, d.x0, d.y0);
        var p1 = OBJ.project(view, d.x1, d.y1);
        if (d.tool === "circle") {
            var rpx = Math.hypot(p1.px - p0.px, p1.py - p0.py);
            ctx.beginPath();
            ctx.arc(p0.px, p0.py, rpx, 0, Math.PI * 2);
            ctx.stroke();
        } else {
            var x = Math.min(p0.px, p1.px);
            var y = Math.min(p0.py, p1.py);
            ctx.strokeRect(x, y, Math.abs(p1.px - p0.px), Math.abs(p1.py - p0.py));
        }
        ctx.restore();
    }

    /** A fit-to-view scale bar: the largest nice span under ~45% of the width. */
    function drawScaleBar(ctx, view) {
        var C = _theColors();
        var avail = view.cssW - 2 * PADDING;
        var span = SCALE_CANDIDATES[SCALE_CANDIDATES.length - 1];
        for (var i = 0; i < SCALE_CANDIDATES.length; i++) {
            if (SCALE_CANDIDATES[i] * view.scale <= avail * SCALE_MAX_FRAC) {
                span = SCALE_CANDIDATES[i];
                break;
            }
        }
        var lenPx = span * view.scale;
        var x0 = SCALE_MARGIN;
        var y0 = view.cssH - SCALE_MARGIN;
        ctx.save();
        ctx.strokeStyle = C.muted;
        ctx.fillStyle = C.muted;
        ctx.lineWidth = 1;
        if (ctx.setLineDash) {
            ctx.setLineDash([]);
        }
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
        ctx.fillText(span + " cm", x0 + lenPx / 2, y0 - SCALE_TICK - 2);
        ctx.restore();
    }

    // ----- repaint orchestration -------------------------------------------

    /**
     * Repaint the single editor canvas: reset to the dpr base transform, clear,
     * then draw the active tab's backdrop + contents + selection + draft + scale
     * bar. Clear+redraw every frame -- never a getImageData/putImageData snapshot
     * (the device-px ghost trap on devicePixelRatio > 1).
     */
    function redraw() {
        var ctx = E.ctx;
        var view = activeView();
        if (!ctx || !view) {
            return;
        }
        ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
        ctx.clearRect(0, 0, view.cssW, view.cssH);
        if (E.state.activeTab === "robot") {
            drawRobotBackdrop(ctx, view);
        } else {
            drawFieldBackdrop(ctx, view);
            drawObjects(ctx, view);
        }
        drawSelection(ctx, view);
        drawDraft(ctx, view);
        drawScaleBar(ctx, view);
    }

    /**
     * Re-measure the canvas, size its backing store to the CSS box x dpr,
     * recompute BOTH tab transforms (the Robot-tab extent may have changed after
     * a load), and repaint. Safe before the canvas is wired.
     */
    function resize() {
        var canvas = E.canvas;
        var ctx = E.ctx;
        if (!canvas || !ctx) {
            return;
        }
        _resolveColors();
        var dpr = window.devicePixelRatio || 1;
        var rect = canvas.getBoundingClientRect();
        var cssW = Math.max(1, Math.round(rect.width));
        var cssH = Math.max(1, Math.round(rect.height));
        canvas.width = Math.round(cssW * dpr);
        canvas.height = Math.round(cssH * dpr);
        E.state.fieldView = computeView(_fieldBounds(), cssW, cssH, dpr);
        E.state.robotView = computeView(_robotBounds(), cssW, cssH, dpr);
        redraw();
    }

    // ----- tab / object / robot state --------------------------------------

    /** Switch the active tab; clears any transient selection/draft/drag. */
    function setTab(name) {
        if (name !== "field" && name !== "robot") {
            return;
        }
        E.state.activeTab = name;
        E.state.selection = null;
        E.state.draft = null;
        E.state.drag = null;
        redraw();
    }

    /** Set the active draw tool ("select" | "rect" | "circle"). */
    function setTool(name) {
        if (name === "select" || name === "rect" || name === "circle") {
            E.state.tool = name;
        }
    }

    /** The object with the given id, or null. */
    function objectById(id) {
        var objs = E.state.objects;
        for (var i = 0; i < objs.length; i++) {
            if (objs[i].id === id) {
                return objs[i];
            }
        }
        return null;
    }

    /** Mint the next unique object id ("oN"), stepping past any loaded ids. */
    function nextId() {
        var max = 0;
        var objs = E.state.objects;
        for (var i = 0; i < objs.length; i++) {
            var m = /^o(\d+)$/.exec(objs[i].id);
            if (m) {
                var n = parseInt(m[1], 10);
                if (n > max) {
                    max = n;
                }
            }
        }
        if (E.state.idSeq > max) {
            max = E.state.idSeq;
        }
        E.state.idSeq = max + 1;
        return "o" + E.state.idSeq;
    }

    /** A fresh default robot config (schema-example zones + 4 drop points). */
    function defaultRobot() {
        return {
            grab_zone: { x_cm: -1, y_cm: 8, w_cm: 6, h_cm: 14 },
            box_zone: { x_cm: -9, y_cm: 0, w_cm: 6, h_cm: 16 },
            drop_points: [
                { x_cm: 4, y_cm: -6 }, { x_cm: 4, y_cm: -2 },
                { x_cm: 4, y_cm: 2 }, { x_cm: 4, y_cm: 6 },
            ],
        };
    }

    // ----- shared editor state (seeded here; siblings attach onto E) --------
    E.state = {
        activeTab: "field",
        tool: "select",
        objects: [],
        robot: defaultRobot(),
        selection: null,   // {kind:"object",id} | {kind:"grab_zone"|"box_zone"} | {kind:"drop",index}
        draft: null,       // {tool,x0,y0,x1,y1} in the active view's world cm
        drag: null,        // {selection, offX, offY} in world cm
        fieldView: null,
        robotView: null,
        idSeq: 0,
    };

    // ----- public core surface ---------------------------------------------
    E.computeView = computeView;
    E.unproject = unproject;
    E.activeView = activeView;
    E.resize = resize;
    E.redraw = redraw;
    E.setTab = setTab;
    E.setTool = setTool;
    E.initMap = initMap;
    E.objectById = objectById;
    E.nextId = nextId;
    E.defaultRobot = defaultRobot;
})();
