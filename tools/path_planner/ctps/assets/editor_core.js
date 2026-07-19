/* CTPS — Layout Editor: core (view transform + field backdrop + tab state).
 *
 * Seeds the editor's own namespace (`window.CTPS_EDITOR`, distinct from the
 * player's `window.CTPS`) and owns the canvas side of the three-tab offline
 * editor (object/cargo plan, T8 -- issue #57; randomized-vision plan, T14 --
 * ADR-060):
 *
 *   - a DUPLICATED `computeView` (the ~40-line world->pixel transform, copied
 *     from renderer.js's private `_computeView` per the plan's Contracts ->
 *     Projection; renderer.js is byte-frozen and NOT given an export), plus the
 *     pixel->world `unproject` INVERSE (editor-only -- the player never drags),
 *   - the Field-tab field backdrop (map image or the vector bounds / obstacles /
 *     nodes) drawn from `const FIELD`,
 *   - the Robot-tab backdrop: the default robot box at `robot_frame_cm` per
 *     side (the true physical chassis footprint), plus the manipulator zones
 *     via `window.CTPS_OBJ.drawZones`, and (T14) the robot-attached vision cone
 *     via `window.CTPS_VISION.drawVisionCone` at an identity pose (the editor
 *     has no robot motion),
 *   - the Mosaic-tab backdrop (T14): the 12-cell mosaic grid, drawn in WORLD
 *     frame (like the Field tab) via `window.CTPS_VISION.mosaicCellCenters` +
 *     `drawMosaicCells` -- the SAME geometry mirror `vision.js` exposes to the
 *     player's live-preview path, reused here rather than re-implemented so the
 *     editor preview agrees with the Python `resolve_cells` authority,
 *   - object rendering via `window.CTPS_OBJ.drawObject`, the selection halo, the
 *     live rubber-band draft, and a fit-to-view scale bar,
 *   - Field/Robot/Mosaic tab state (incl. default `mosaic` + `vision` blocks,
 *     T14) + the single clear+redraw repaint (never a getImageData/
 *     putImageData snapshot -- the device-px ghost trap).
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
    var VIS = window.CTPS_VISION; // vision.js (inlined before this file): mosaic/cone geometry+draw.

    // ----- layout / paint constants (named, no magic literals inline) ------
    var PADDING = 24;            // px inset of the drawn field within the stage box
    var ROBOT_FALLBACK_CM = 9;   // robot half-side (cm) when the field omits a radius
    var ROBOT_MARGIN_CM = 5;     // extra world margin around the Robot-tab contents
    var ROBOT_ZOOM_DEFAULT = 0.65; // render-only Robot-tab zoom (1.0 == fit-to-view)
    var ROBOT_ZOOM_MIN = 0.25;   // Robot-tab zoom lower clamp (matches the slider)
    var ROBOT_ZOOM_MAX = 2.0;    // Robot-tab zoom upper clamp (matches the slider)
    var ROBOT_OUTLINE_W = 2;     // robot-box outline stroke width (px)
    var ROBOT_CORNER = 0.3;      // robot-box rounded-corner radius (frac of half-side)
    var ROBOT_ARROW_TIP = 0.28;  // heading-arrow tip x (frac of half-side)
    var ROBOT_ARROW_BACK = 0.18; // heading-arrow base setback (frac of half-side)
    var ROBOT_ARROW_HALF = 0.2;  // heading-arrow base half-width (frac of half-side)
    var AXLE_DOT_R = 4;          // robot-axle-centre marker radius (px)
    var AXLE_LABEL_DX = 7;       // axle-centre label x-offset from the dot (px)
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

    // ----- mosaic / vision-cone defaults (T14 -- ADR-060) -------------------
    var MOSAIC_ROWS_DEFAULT = 4;         // default grid rows (rows*cols MUST == 12)
    var MOSAIC_COLS_DEFAULT = 3;         // default grid cols
    var MOSAIC_CELL_W_CM_DEFAULT = 5.0;  // default cell width (cm)
    var MOSAIC_CELL_H_CM_DEFAULT = 5.0;  // default cell height (cm)
    var MOSAIC_ROW_GAP_CM_DEFAULT = 1.0; // default uniform row gap (cm)
    var MOSAIC_COL_GAP_CM_DEFAULT = 1.0; // default uniform column gap (cm)
    var MOSAIC_HEADING_DEG_DEFAULT = 0.0; // default mosaic orientation (deg)
    var MOSAIC_CELL_COUNT = 12;          // rows*cols MUST equal this (Config::VISION_CELL_COUNT)
    var MOSAIC_MARGIN_CM = 10;           // extra world margin around the mosaic on the Mosaic tab
    var VISION_ORIGIN_X_CM_DEFAULT = 12.0; // default cone apex, forward of the robot centre (cm)
    var VISION_ORIGIN_Y_CM_DEFAULT = 0.0;  // default cone apex lateral offset (cm)
    var VISION_HALF_ANGLE_DEG_DEFAULT = 25.0; // default half-FOV (deg)
    var VISION_RANGE_CM_DEFAULT = null;    // default range: null == unbounded
    var VISION_CONE_PREVIEW_REACH_CM = 60; // Robot-tab bounds-fit reach when range_cm is unbounded

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
            axle: _cssVar("--ctps-axle"),
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

    /**
     * The robot half-side in world cm. Prefers `robot_frame_cm / 2` (the TRUE
     * physical chassis footprint, a 25 cm square), falls back to the C-space
     * `robot_radius_cm` for legacy fields, then to a fixed fallback.
     */
    function _robotHalfCm() {
        var f = _field();
        var frame = f && f.robot_frame_cm;
        if (typeof frame === "number" && frame > 0) {
            return frame / 2;
        }
        var r = f && f.robot_radius_cm;
        return (typeof r === "number" && r > 0) ? r : ROBOT_FALLBACK_CM;
    }

    /**
     * The Robot-tab world extent `[x0,y0,x1,y1]` (robot-LOCAL cm): the union of
     * the robot box, both zones, every drop point, and (T14) the vision cone's
     * apex + its two edge-ray tips (bounded reach, or a fixed preview reach when
     * `range_cm` is null/unbounded so an infinite cone does not blow out the
     * view), plus a margin. The Robot tab renders the local frame directly
     * (pose identity), so local cm ARE the view's world cm.
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
        var cone = E.state.vision;
        if (cone) {
            var reach = (typeof cone.range_cm === "number" && cone.range_cm > 0)
                ? cone.range_cm : VISION_CONE_PREVIEW_REACH_CM;
            var half_rad = cone.half_angle_deg * Math.PI / 180;
            ext(cone.origin_x_cm, cone.origin_y_cm);
            ext(cone.origin_x_cm + reach * Math.cos(half_rad),
                cone.origin_y_cm + reach * Math.sin(half_rad));
            ext(cone.origin_x_cm + reach * Math.cos(-half_rad),
                cone.origin_y_cm + reach * Math.sin(-half_rad));
        }
        return [
            box.minX - ROBOT_MARGIN_CM, box.minY - ROBOT_MARGIN_CM,
            box.maxX + ROBOT_MARGIN_CM, box.maxY + ROBOT_MARGIN_CM,
        ];
    }

    /**
     * The mosaic's world-cm bounding box `{minX,minY,maxX,maxY}` -- the union of
     * every resolved cell centre +/- half a cell -- or null when no mosaic is
     * present or the vision helpers are unavailable. Single source shared by the
     * Mosaic-tab bounds fit, the drag hit-test (editor_tools.js `_hitMosaic`),
     * and the selection halo so all three agree on the mosaic's footprint.
     */
    function mosaicBBox() {
        var mosaic = E.state.mosaic;
        if (!mosaic || !VIS || !VIS.mosaicCellCenters) {
            return null;
        }
        var centers = VIS.mosaicCellCenters(mosaic);
        if (!centers.length) {
            return null;
        }
        var halfW = mosaic.cell_w_cm / 2;
        var halfH = mosaic.cell_h_cm / 2;
        var bb = { minX: Infinity, minY: Infinity, maxX: -Infinity, maxY: -Infinity };
        for (var i = 0; i < centers.length; i++) {
            var c = centers[i];
            if (c.x_cm - halfW < bb.minX) { bb.minX = c.x_cm - halfW; }
            if (c.x_cm + halfW > bb.maxX) { bb.maxX = c.x_cm + halfW; }
            if (c.y_cm - halfH < bb.minY) { bb.minY = c.y_cm - halfH; }
            if (c.y_cm + halfH > bb.maxY) { bb.maxY = c.y_cm + halfH; }
        }
        return bb;
    }

    /**
     * The Mosaic-tab world extent `[x0,y0,x1,y1]` (WORLD cm): the union of the
     * field bounds (so the field map stays visible for placement context) and
     * the mosaic's own resolved-cell extent (so an off-centre or oversized
     * mosaic still fits fully in view -- the task's "extend the editor bounds"
     * step), plus a margin.
     */
    function _mosaicBounds() {
        var b = _fieldBounds();
        var box = { minX: b[0], minY: b[1], maxX: b[2], maxY: b[3] };
        var bb = mosaicBBox();
        if (bb) {
            if (bb.minX < box.minX) { box.minX = bb.minX; }
            if (bb.maxX > box.maxX) { box.maxX = bb.maxX; }
            if (bb.minY < box.minY) { box.minY = bb.minY; }
            if (bb.maxY > box.maxY) { box.maxY = bb.maxY; }
        }
        return [
            box.minX - MOSAIC_MARGIN_CM, box.minY - MOSAIC_MARGIN_CM,
            box.maxX + MOSAIC_MARGIN_CM, box.maxY + MOSAIC_MARGIN_CM,
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
     * Scale a freshly-fit Robot-tab view by the session `robotZoom` multiplier.
     * This is RENDER-ONLY: it never touches `robot_radius_cm` or any robot-local
     * cm coordinate, only the on-screen transform. The robot-bounds centre stays
     * pinned to the stage centre, so BOTH the draw and the pixel->world
     * `unproject` inverse read this one scaled transform -- dragging/selecting
     * stays correct at any zoom. A multiplier of 1.0 reproduces the plain
     * fit-to-view exactly. Mutates + returns the passed view.
     *
     * @param view a fit transform from `computeView(_robotBounds(), ...)`.
     * @return the same `view`, its `scale` / `padX` / `padY` re-scaled + centred.
     */
    function _applyRobotZoom(view) {
        var zoom = E.state.robotZoom;
        if (!(zoom > 0) || zoom === 1) {
            return view;
        }
        var scale = view.scale * zoom;
        view.scale = scale;
        // Re-centre so the robot-bounds midpoint holds the stage centre: at
        // zoom 1 this reduces to computeView's own centred pad, and both project
        // and unproject stay consistent because they share this same view.
        view.padX = view.cssW / 2 - (view.worldW / 2) * scale;
        view.padY = view.cssH / 2 - (view.worldH / 2) * scale;
        return view;
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
        if (E.state.activeTab === "robot") {
            return E.state.robotView;
        }
        if (E.state.activeTab === "mosaic") {
            return E.state.mosaicView;
        }
        return E.state.fieldView;
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

    /** Project a world extent `[x0,y0,x1,y1]` to a top-left/size canvas-pixel rect. */
    function _projectBounds(view, b) {
        var tl = OBJ.project(view, b[0], b[3]); // (x0, y1) -> top-left
        var br = OBJ.project(view, b[2], b[1]); // (x1, y0) -> bottom-right
        return { x: tl.px, y: tl.py, w: br.px - tl.px, h: br.py - tl.py };
    }

    /** The current view's own extent (`view.bounds`) as a canvas-pixel rect. */
    function _boundsRect(view) {
        return _projectBounds(view, view.bounds);
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

    /**
     * Draw the robot AXLE CENTRE as a red dot + label on the Robot tab. The
     * editor draws the body box + zones + cone at the body-centre origin (0,0)
     * -- everything the operator authors is body-centre-relative, exactly as the
     * player draws them at `axleToBodyPose` (the body pose). The rotation axle --
     * the point that sits on a graph node and that the robot pivots about -- is
     * offset from the body centre by the NEGATED `axle_offset`: the player uses
     * `body = axle + Rot(h)*offset`, so `axle = body - Rot(h)*offset`, and the
     * editor pose is identity (h = 0), leaving the axle at local
     * `(-forward_cm, -lateral_cm)`. With no offset the axle coincides with the
     * body centre (dot at the origin). Reads the offset defensively -- a
     * missing/NaN component is 0 -- so it never throws on a partial state.
     */
    function drawAxleCenter(ctx, view) {
        var off = E.state.axleOffset || {};
        var fwd = (typeof off.forward_cm === "number" && isFinite(off.forward_cm))
            ? off.forward_cm : 0;
        var lat = (typeof off.lateral_cm === "number" && isFinite(off.lateral_cm))
            ? off.lateral_cm : 0;
        var C = _theColors();
        var p = OBJ.project(view, -fwd, -lat);
        ctx.save();
        ctx.fillStyle = C.axle;
        ctx.beginPath();
        ctx.arc(p.px, p.py, AXLE_DOT_R, 0, Math.PI * 2);
        ctx.fill();
        ctx.font = LABEL_FONT;
        ctx.textAlign = "left";
        ctx.textBaseline = "middle";
        ctx.fillText("robot axle center", p.px + AXLE_LABEL_DX, p.py);
        ctx.restore();
    }

    /** Draw the Robot-tab backdrop: robot box + zones overlay + labels + vision cone + axle centre. */
    function drawRobotBackdrop(ctx, view) {
        drawRobotBox(ctx, view);
        var rob = E.state.robot;
        if (rob) {
            // Pose identity -> the zones' robot-local cm map straight to world cm.
            OBJ.drawZones(ctx, view, { x: 0, y: 0, h: 0 }, rob, OBJ.project);
            _drawZoneLabels(ctx, view, rob);
        }
        drawRobotVisionCone(ctx, view);
        drawAxleCenter(ctx, view);
    }

    /**
     * Draw the robot-attached vision cone on the Robot tab, reusing
     * `CTPS_VISION.drawVisionCone` (the SAME predicate-adjacent draw the player
     * uses) at an IDENTITY pose: the editor has no robot motion, so the cone is
     * drawn at the robot's own local origin, heading 0 -- exactly the pose-0
     * convention `drawRobotBackdrop`'s zones already use. No-op when `VIS` (the
     * `vision.js` export, inlined before this file) or the cone state is absent.
     */
    function drawRobotVisionCone(ctx, view) {
        var cone = E.state.vision;
        if (!cone || !VIS || !VIS.drawVisionCone) {
            return;
        }
        VIS.drawVisionCone(ctx, view, { x: 0, y: 0, h: 0 }, cone, OBJ.project);
    }

    /**
     * Draw the Mosaic-tab backdrop: the field bounds outline (for spatial
     * context, matching the Field tab's frame) plus the 12-cell mosaic grid,
     * reusing `CTPS_VISION.mosaicCellCenters` (the geometry mirror) +
     * `drawMosaicCells` (the shared draw) -- the SAME functions the player uses
     * for its live preview, so the editor never re-implements the resolve_cells
     * geometry (Notes for Implementer -> geometry mirror). Pinned cells (a
     * non-null `mosaic.cells[i]`) are reflected as filled colour; unset cells
     * draw as a bare outline (no roll to preview in the editor).
     */
    function drawMosaicBackdrop(ctx, view) {
        var C = _theColors();
        // The field map + nodes as a PLACEMENT backdrop so the mosaic seats
        // against the real field (the "show the map and output the mosaic where I
        // want it" requirement). The surface is drawn to the FIELD bounds
        // (projected in the mosaic view), NOT `view.bounds` -- the mosaic view is
        // the wider field-union-mosaic extent, so projecting `view.bounds` would
        // stretch the map past the field it depicts. Nodes reuse the Field-tab
        // draw (world coords, correct under any view).
        var r = _projectBounds(view, _fieldBounds());
        _drawFieldSurface(ctx, view, r);
        ctx.save();
        ctx.strokeStyle = C.borderStrong;
        ctx.lineWidth = BOUNDS_W;
        ctx.strokeRect(r.x, r.y, r.w, r.h);
        ctx.restore();
        _drawNodes(ctx, view);

        var mosaic = E.state.mosaic;
        if (!mosaic || !VIS || !VIS.mosaicCellCenters || !VIS.drawMosaicCells) {
            return;
        }
        var centers = VIS.mosaicCellCenters(mosaic);
        var cells = new Array(centers.length);
        for (var i = 0; i < centers.length; i++) {
            var c = centers[i];
            cells[i] = {
                index: c.index,
                x_cm: c.x_cm,
                y_cm: c.y_cm,
                color: mosaic.cells ? mosaic.cells[c.index] : null,
            };
        }
        var meta = (typeof MOSAIC_META !== "undefined") ? MOSAIC_META : null;
        VIS.drawMosaicCells(ctx, view, cells, OBJ.project, meta, mosaic.cell_w_cm);
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
        } else if (s.kind === "mosaic") {
            // A dashed halo around the whole mosaic footprint while it is the
            // drag target -- the visual grab affordance for placing it on the map.
            var mb = mosaicBBox();
            if (mb) {
                var mtl = OBJ.project(view, mb.minX, mb.maxY);
                var mbr = OBJ.project(view, mb.maxX, mb.minY);
                ctx.strokeRect(mtl.px - SEL_PAD, mtl.py - SEL_PAD,
                    (mbr.px - mtl.px) + 2 * SEL_PAD, (mbr.py - mtl.py) + 2 * SEL_PAD);
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
        } else if (E.state.activeTab === "mosaic") {
            drawMosaicBackdrop(ctx, view);
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
     * recompute ALL THREE tab transforms (the Robot-tab extent may have changed
     * after a load; the Mosaic-tab extent likewise depends on the current mosaic
     * state), and repaint. Safe before the canvas is wired.
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
        E.state.robotView = _applyRobotZoom(
            computeView(_robotBounds(), cssW, cssH, dpr)
        );
        E.state.mosaicView = computeView(_mosaicBounds(), cssW, cssH, dpr);
        redraw();
    }

    // ----- tab / object / robot state --------------------------------------

    /** Switch the active tab; clears any transient selection/draft/drag. */
    function setTab(name) {
        if (name !== "field" && name !== "robot" && name !== "mosaic") {
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

    /** Clamp a render zoom multiplier into the supported Robot-tab range. */
    function clampRobotZoom(zoom) {
        if (typeof zoom !== "number" || !isFinite(zoom)) {
            return E.state.robotZoom;
        }
        if (zoom < ROBOT_ZOOM_MIN) {
            return ROBOT_ZOOM_MIN;
        }
        if (zoom > ROBOT_ZOOM_MAX) {
            return ROBOT_ZOOM_MAX;
        }
        return zoom;
    }

    /**
     * Set the render-only Robot-tab zoom (clamped), re-fit + re-scale the robot
     * view around its centre, and repaint. This is EDITOR-SESSION render state
     * ONLY -- it is never serialised into layout.json and never changes any
     * robot-local cm. The robot view is re-derived from a FRESH fit before the
     * multiplier is applied, so the zoom can never compound across calls.
     *
     * @param zoom the desired multiplier (1.0 == fit-to-view).
     * @return the clamped multiplier actually applied.
     */
    function setRobotZoom(zoom) {
        E.state.robotZoom = clampRobotZoom(zoom);
        var v = E.state.robotView;
        if (v) {
            E.state.robotView = _applyRobotZoom(
                computeView(_robotBounds(), v.cssW, v.cssH, v.dpr)
            );
        }
        redraw();
        return E.state.robotZoom;
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

    /**
     * The colour vocabulary the Mosaic tab authors against: `MOSAIC_META.names`
     * (the unconditional editor-emitted colour constant, single-sourced from
     * `mosaic.py` -- ADR-060) when present, else an empty list (a legacy/served
     * page missing the constant degrades to no palette rather than a
     * hand-duplicated colour name).
     */
    function _paletteNames() {
        if (typeof MOSAIC_META !== "undefined" && MOSAIC_META && MOSAIC_META.names) {
            return MOSAIC_META.names.slice();
        }
        return [];
    }

    /**
     * Even-as-possible per-colour quotas for `count` NULL cells spread over
     * `names.length` palette colours, summing to EXACTLY `count` (the leading
     * colours absorb the +1 remainder) -- e.g. 12 cells over 4 colours -> 3
     * each; 12 over 3 -> 4 each.
     */
    function _evenCounts(names, count) {
        var counts = {};
        if (!names || names.length === 0) {
            return counts;
        }
        var base = Math.floor(count / names.length);
        var extra = count % names.length;
        for (var i = 0; i < names.length; i++) {
            counts[names[i]] = base + (i < extra ? 1 : 0);
        }
        return counts;
    }

    /**
     * A fresh default `mosaic` block (ADR-060): seated at the FIELD CENTRE
     * (derived from the current field bounds' midpoint -- never a hardcoded
     * coordinate, so the default stays sane across fields), a 4x3 grid, a
     * reasonable cell size + uniform gaps, `mode:"random"` over the full
     * `MOSAIC_META` palette, every cell unset (`null` -- rolled), and `counts`
     * spread evenly over the palette summing to the full 12 cells.
     */
    function defaultMosaic() {
        var b = _fieldBounds();
        var centerX = (b[0] + b[2]) / 2;
        var centerY = (b[1] + b[3]) / 2;
        var palette = _paletteNames();
        var cells = new Array(MOSAIC_CELL_COUNT);
        for (var i = 0; i < MOSAIC_CELL_COUNT; i++) {
            cells[i] = null;
        }
        return {
            mode: "random",
            center_x_cm: centerX,
            center_y_cm: centerY,
            heading_deg: MOSAIC_HEADING_DEG_DEFAULT,
            rows: MOSAIC_ROWS_DEFAULT,
            cols: MOSAIC_COLS_DEFAULT,
            cell_w_cm: MOSAIC_CELL_W_CM_DEFAULT,
            cell_h_cm: MOSAIC_CELL_H_CM_DEFAULT,
            row_gap_cm: MOSAIC_ROW_GAP_CM_DEFAULT,
            col_gap_cm: MOSAIC_COL_GAP_CM_DEFAULT,
            palette: palette,
            cells: cells,
            counts: _evenCounts(palette, MOSAIC_CELL_COUNT),
            seed: null,
        };
    }

    /**
     * A fresh default `robot.vision` cone: origin forward of the robot centre,
     * on the robot's midline, a ~25 deg half-angle, and an unbounded (`null`)
     * range -- a reasonable starting FOV an author narrows/bounds per the real
     * camera mount.
     */
    function defaultVisionCone() {
        return {
            origin_x_cm: VISION_ORIGIN_X_CM_DEFAULT,
            origin_y_cm: VISION_ORIGIN_Y_CM_DEFAULT,
            half_angle_deg: VISION_HALF_ANGLE_DEG_DEFAULT,
            range_cm: VISION_RANGE_CM_DEFAULT,
        };
    }

    // ----- shared editor state (seeded here; siblings attach onto E) --------
    E.state = {
        activeTab: "field",
        tool: "select",
        objects: [],
        robot: defaultRobot(),
        mosaic: defaultMosaic(),
        vision: defaultVisionCone(),
        selection: null,   // {kind:"object",id} | {kind:"grab_zone"|"box_zone"} | {kind:"drop",index}
        draft: null,       // {tool,x0,y0,x1,y1} in the active view's world cm
        drag: null,        // {selection, offX, offY} in world cm
        fieldView: null,
        robotView: null,
        mosaicView: null,
        robotZoom: ROBOT_ZOOM_DEFAULT, // render-only Robot-tab zoom (never serialised)
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
    E.setRobotZoom = setRobotZoom;
    E.initMap = initMap;
    E.objectById = objectById;
    E.nextId = nextId;
    E.defaultRobot = defaultRobot;
    // Mosaic / vision-cone state (T14 -- ADR-060). T15 (property-panel editing +
    // the pin-grid cycle) and T16 (serialize/ingest) read/mutate `E.state.mosaic`
    // / `E.state.vision` directly (matching how T15 already mutates
    // `E.state.robot` in place) and call `E.redraw()` after; the two factories
    // + the palette accessor are exposed so a "reset to default" affordance or
    // ingest-side fallback never re-derives the field-centre-seat rule.
    E.defaultMosaic = defaultMosaic;
    E.defaultVisionCone = defaultVisionCone;
    E.mosaicPaletteNames = _paletteNames;
    // The mosaic footprint (world cm), shared with editor_tools.js's drag
    // hit-test so click-to-grab and the selection halo use one definition.
    E.mosaicBBox = mosaicBBox;
})();
