/* CTPS — Clear Trajectory Prediction Simulator: randomized-vision predicate,
 * layers, and read HUD (randomized-vision plan, T11 — ADR-060).
 *
 * The player-side of the CTPS "randomized vision" feature: the 4x3 colour mosaic
 * seated in the world, a robot-attached forward vision cone that moves with the
 * playhead, and a per-frame `readMosaic()` PREDICTION HUD (true iff all 12 cells
 * fall inside the cone). This file has four clearly separated concerns:
 *
 *   1. GEOMETRY / PREDICATE (the load-bearing, Python-mirrored math):
 *      - `visibleCells(pose, visionCfg, cells) -> bool[12]` — the per-cell
 *        cone-visibility predicate. Point-in-sector: a cell is visible iff it is
 *        forward of the cone apex, within `half_angle_deg` of robot-forward, and
 *        within `range_cm` (any distance when `range_cm` is null). The apex is
 *        `localToWorld(pose, origin_x_cm, origin_y_cm)`; each cell's world delta
 *        is expressed in robot-local by `Rot(-pose.h)`. This is a byte-faithful
 *        mirror of the Python predicate in `test_ctps_mosaic.py` (the No-JS-runner
 *        note): degrees on both sides of the angle test, same sign convention.
 *      - `readMosaicWouldSucceed(bools) -> bool` = all true.
 *      - `mosaicCellCenters(geom) -> [{index,x_cm,y_cm}]` — the EDITOR-preview
 *        mirror of Python `resolve_cells` geometry (T14 live preview); the SAME
 *        formula T18 pins, so the editor draw agrees with the authoritative
 *        Python resolve.
 *
 *   2. DRAWS (transform-agnostic, reuse `CTPS_OBJ.project` / `localToWorld`):
 *      - `drawMosaicCells(ctx, view, cells, project)` — the 12 world cells as
 *        squares (filled from `MOSAIC_META.hex[cell.color]` when a roll is
 *        present, else a bare outline for the no-roll grid).
 *      - `drawVisionCone(ctx, view, pose, visionCfg, project)` — the cone from
 *        the apex out to `range_cm` (or a long ray when unbounded) at
 *        +/- `half_angle_deg` from robot-forward, following the pose each frame.
 *
 *   3. LAYERS (ADR-053 registry, on the `trail` canvas like the cargo layers):
 *      - a `mosaic` layer (static world cells) and a `vision-cone` layer that
 *        moves with the robot each frame off `CTPS.engine.poseAt(state.t)` — NOT
 *        the resolve-once cargo scene. Both no-op when their DATA is absent and
 *        respect the `data-layer="mosaic"` / `data-layer="vision-cone"` chips.
 *
 *   4. HUD (the VISIBLE `#ctps-vision` rail panel, never the hidden `#ctps-hud`):
 *      each frame recompute `visibleCells(poseAt(t), cone, cells)`, show `k/12`
 *      in `.ctps-vision-hud-count`, name a cell outside the cone in
 *      `.ctps-vision-hud-detail` when `k<12`, apply the success modifier at
 *      12/12, and surface the no-roll / geometry-changed / no-mosaic /
 *      no-cone badges in `#ctps-vision-badges`.
 *
 * DATA SOURCES (all guarded for undefined — the feature is inactive on a
 * mosaic-free page): the emitted `MOSAIC` global (resolved cells + roll/geom
 * flags), the `MOSAIC_META` global (`{names, hex}` colour vocabulary), and the
 * cone from the loaded `robot.vision` (`store.visionCone`). On the ADR-056 upload
 * leg, `store.loadLayout` re-derives the embed from an uploaded layout's own
 * `mosaic`/`robot.vision` blocks via `CTPS.vision.embedFromLayout` (defined
 * here) so an uploaded mosaic renders on a mosaic-free-emitted page.
 *
 * Consumes the pinned engine via `CTPS.engine` (poseAt) and the store via
 * `CTPS.store`. Inlined AFTER `objects_draw.js` (so `CTPS_OBJ` exists) and after
 * `store.js` (so `CTPS.store.setVisionEmbed` exists). Vanilla ES5 — var/function
 * only, works offline. No network.
 *
 * Coordinate convention (matches renderer.js / cargo.js): centimetres, degrees
 * in [0, 360), 0 deg = +x, CCW positive, world +Y up. The mosaic block is
 * WORLD-frame; the vision cone is robot-LOCAL (+x forward, +y left).
 */

"use strict";

(function () {
    var CTPS = window.CTPS;
    var OBJ = window.CTPS_OBJ;

    // ----- constants (named, no magic literals inline) ---------------------
    var CELL_COUNT = 12;             // the fixed 4x3 mosaic cell count (Config::VISION_CELL_COUNT)
    var CONE_UNBOUNDED_CM = 100000;  // ray length (cm) drawn for an unbounded (null range) cone
    var CONE_STROKE_W = 1.5;         // cone outline stroke width (px)
    var CONE_FILL_ALPHA = 0.12;      // translucent cone fill opacity
    var CELL_STROKE_W = 1.5;         // mosaic cell outline stroke width (px)
    var CELL_FILL_ALPHA = 0.85;      // filled (rolled) mosaic cell opacity
    var CELL_OUTLINE_ALPHA = 0.7;    // no-roll cell outline opacity
    var DEG = Math.PI / 180;

    // ----- token colour resolution (cached; static in this app) ------------
    // Same discipline as objects_draw.js: resolve the cone token colours once
    // from the inlined stylesheet on first draw and cache them, so a per-frame
    // cone redraw never re-hits getComputedStyle.
    var _tokens = null;

    /** Read one CSS custom property off the document root, trimmed ("" when unset). */
    function _cssVar(name) {
        var v = getComputedStyle(document.documentElement).getPropertyValue(name);
        return v ? v.replace(/^\s+|\s+$/g, "") : "";
    }

    /** Resolve + cache the cone/cell token colours. */
    function _theTokens() {
        if (!_tokens) {
            _tokens = {
                accent: _cssVar("--ctps-accent"),
                faint: _cssVar("--ctps-faint"),
                text: _cssVar("--ctps-text"),
            };
        }
        return _tokens;
    }

    // ----- geometry helpers (reuse objects_draw.js, one copy of the math) --
    // Robot-local -> world reuses objects_draw.js's `CTPS_OBJ.localToWorld`
    // (loaded before this file): ONE copy of the rotation math on the page so a
    // sign/handedness fix can never desync the apex here from the cone/cell draw.
    var _localToWorld = OBJ ? OBJ.localToWorld : null;

    /** Robot heading (degrees) off a pose `{x, y, h}`; 0 when the field is missing. */
    function _poseHeading(pose) {
        return (pose && typeof pose.h === "number") ? pose.h : 0;
    }

    // ----- 1. predicate + editor-mirror geometry ---------------------------

    /**
     * The per-cell cone-visibility predicate: a 12-element boolean array (order =
     * cell index) — a cell is visible iff it is FORWARD of the cone apex, within
     * `half_angle_deg` of robot-forward, and within `range_cm` (or any distance
     * when `range_cm` is null/absent).
     *
     * BYTE-FAITHFUL mirror of the Python predicate in test_ctps_mosaic.py (the
     * No-JS-runner note): apex = `localToWorld(pose, origin_x_cm, origin_y_cm)`;
     * each cell's world delta `(cell.x - apex.x, cell.y - apex.y)` is rotated by
     * `Rot(-heading)` into robot-local `(lx, ly)`; then
     *   visible = (lx > 0)
     *          && (abs(degrees(atan2(ly, lx))) <= half_angle_deg)
     *          && (range_cm == null || hypot(lx, ly) <= range_cm)
     * — degrees on BOTH sides of the angle test. `Rot(-h) . (dx, dy) =
     * (dx*cos h + dy*sin h, -dx*sin h + dy*cos h)`.
     *
     * @param pose the robot pose `{x, y, h}` (cm, cm, deg) — e.g. poseAt(t).
     * @param visionCfg the robot-local cone `{origin_x_cm, origin_y_cm,
     *     half_angle_deg, range_cm}` (`range_cm` null/absent = unbounded).
     * @param cells the resolved mosaic cells (`[{x_cm, y_cm, ...}]`), read in
     *     order — index i of the result is cell i.
     * @return a boolean array, one entry per cell (empty when either the cone or
     *     the cells are missing).
     */
    function visibleCells(pose, visionCfg, cells) {
        var results = [];
        if (!pose || !visionCfg || !cells || !_localToWorld) {
            return results;
        }
        var apex = _localToWorld(pose, visionCfg.origin_x_cm, visionCfg.origin_y_cm);
        var h = _poseHeading(pose);
        var rad = h * DEG;
        var cosT = Math.cos(rad);
        var sinT = Math.sin(rad);
        var halfAngle = visionCfg.half_angle_deg;
        // null OR undefined range => unbounded (the Python mirror's `is None`).
        var range = (visionCfg.range_cm === null || visionCfg.range_cm === undefined)
            ? null : visionCfg.range_cm;

        for (var i = 0; i < cells.length; i++) {
            var cell = cells[i];
            var dx = cell.x_cm - apex.x;
            var dy = cell.y_cm - apex.y;
            // Rot(-h) . (dx, dy) -> robot-local.
            var lx = dx * cosT + dy * sinT;
            var ly = -dx * sinT + dy * cosT;

            var visible = lx > 0;
            if (visible) {
                var angleDeg = Math.atan2(ly, lx) / DEG;
                visible = Math.abs(angleDeg) <= halfAngle;
            }
            if (visible && range !== null) {
                visible = Math.hypot(lx, ly) <= range;
            }
            results.push(visible);
        }
        return results;
    }

    /**
     * True iff every entry of `bools` is true — the geometric read PREDICTION
     * (read succeeds iff all 12 cells fall inside the cone, per ADR-060). An
     * empty array (no cells / no cone) reads as false: there is nothing to read.
     *
     * @param bools the per-cell visibility array from `visibleCells`.
     * @return true iff `bools` is non-empty and every entry is true.
     */
    function readMosaicWouldSucceed(bools) {
        if (!bools || bools.length === 0) {
            return false;
        }
        for (var i = 0; i < bools.length; i++) {
            if (!bools[i]) {
                return false;
            }
        }
        return true;
    }

    /**
     * The editor-preview MIRROR of Python `resolve_cells` GEOMETRY: the 12 world
     * cell centres for a mosaic geometry block (NO colours — the editor colours
     * them itself). Implements the SAME formula the authoritative Python resolve
     * (and T18) pins:
     *   i        = row * cols + col          (row 0 at +y "top", col 0 at left)
     *   local_x  = (col - (cols - 1) / 2) * (cell_w_cm + col_gap_cm)
     *   local_y  = ((rows - 1) / 2 - row)   * (cell_h_cm + row_gap_cm)
     *   world    = (center_x_cm, center_y_cm) + Rot(heading_deg) . (local_x, local_y)
     * where Rot(t) . (x, y) = (x cos t - y sin t, x sin t + y cos t) (CCW).
     *
     * This is the NON-load-bearing editor live-preview path (T14); the player
     * draws the Python-resolved `MOSAIC` cells directly. It must AGREE with
     * `resolve_cells` on the same pinned vectors (T24 eyeball).
     *
     * @param geom a mosaic geometry `{rows, cols, cell_w_cm, cell_h_cm,
     *     row_gap_cm, col_gap_cm, center_x_cm, center_y_cm, heading_deg}`.
     * @return `[{index, x_cm, y_cm}]` in ascending index order (empty when the
     *     geometry is missing or degenerate).
     */
    function mosaicCellCenters(geom) {
        var cells = [];
        if (!geom) {
            return cells;
        }
        var rows = geom.rows | 0;
        var cols = geom.cols | 0;
        if (rows <= 0 || cols <= 0) {
            return cells;
        }
        var cellW = +geom.cell_w_cm || 0;
        var cellH = +geom.cell_h_cm || 0;
        var rowGap = +geom.row_gap_cm || 0;
        var colGap = +geom.col_gap_cm || 0;
        var centerX = +geom.center_x_cm || 0;
        var centerY = +geom.center_y_cm || 0;
        var headingDeg = +geom.heading_deg || 0;

        var rad = headingDeg * DEG;
        var cosT = Math.cos(rad);
        var sinT = Math.sin(rad);
        var colPitch = cellW + colGap;
        var rowPitch = cellH + rowGap;
        var colMid = (cols - 1) / 2;
        var rowMid = (rows - 1) / 2;

        for (var row = 0; row < rows; row++) {
            for (var col = 0; col < cols; col++) {
                var index = row * cols + col;
                var localX = (col - colMid) * colPitch;
                var localY = (rowMid - row) * rowPitch; // row 0 at +y "top"
                var worldX = centerX + (localX * cosT - localY * sinT);
                var worldY = centerY + (localX * sinT + localY * cosT);
                cells.push({ index: index, x_cm: worldX, y_cm: worldY });
            }
        }
        return cells;
    }

    // ----- 2. draws --------------------------------------------------------

    /**
     * Draw the mosaic cells through `project`: each cell is a small square
     * centred on its world position with side = `cell_w_cm` x `view.scale`.
     * A cell carrying a `color` (a roll is present) is filled with
     * `MOSAIC_META.hex[color]` (looked up in `meta`); a cell with no colour (the
     * no-roll grid-outline case) is a bare stroked square. `meta` may be null (an
     * uploaded mosaic can arrive before MOSAIC_META); an unknown colour degrades
     * to an outline, never a hard-coded hex.
     *
     * The square is drawn axis-aligned in WORLD space (the mosaic's own heading
     * already baked the cell centres into world by resolve_cells) — a plain
     * projected rect, so it reads as the field-frame grid the firmware indexes.
     *
     * @param ctx the target 2D context.
     * @param view the world->pixel transform (renderer's or editor's).
     * @param cells the resolved cells (`[{index, x_cm, y_cm, color?}]`).
     * @param project the projection fn (pass `CTPS_OBJ.project`).
     * @param meta the `{hex:{NAME:"#.."}}` colour vocabulary, or null.
     * @param cellWcm the cell side in cm (for the square size); a non-positive
     *     value degrades to a small fixed size so a mis-sized cell still shows.
     */
    function drawMosaicCells(ctx, view, cells, project, meta, cellWcm) {
        if (!cells || !project) {
            return;
        }
        var tokens = _theTokens();
        var sideCm = (typeof cellWcm === "number" && cellWcm > 0) ? cellWcm : 3;
        var sidePx = sideCm * view.scale;
        var hex = (meta && meta.hex) || null;
        ctx.save();
        ctx.lineWidth = CELL_STROKE_W;
        for (var i = 0; i < cells.length; i++) {
            var cell = cells[i];
            var c = project(view, cell.x_cm, cell.y_cm);
            var fill = (hex && cell.color) ? hex[cell.color] : null;
            ctx.beginPath();
            ctx.rect(c.px - sidePx / 2, c.py - sidePx / 2, sidePx, sidePx);
            if (fill) {
                ctx.globalAlpha = CELL_FILL_ALPHA;
                ctx.fillStyle = fill;
                ctx.fill();
                ctx.globalAlpha = 1;
                ctx.strokeStyle = tokens.text || fill;
                ctx.stroke();
            } else {
                // No roll (or unknown colour): outline only, no synthesized fill.
                ctx.globalAlpha = CELL_OUTLINE_ALPHA;
                ctx.strokeStyle = tokens.faint || tokens.text || "";
                ctx.stroke();
                ctx.globalAlpha = 1;
            }
        }
        ctx.restore();
    }

    /**
     * Draw the robot-attached vision cone through `project`: a filled + stroked
     * wedge from the apex (`localToWorld(pose, origin_x_cm, origin_y_cm)`) out to
     * `range_cm` (or a long ray `CONE_UNBOUNDED_CM` when `range_cm` is
     * null/absent) at +/- `half_angle_deg` from robot-forward. The two edge rays
     * are built in ROBOT-LOCAL then mapped to world through the pose, so the
     * whole cone follows the robot heading each frame.
     *
     * A missing pose / cfg / project degrades to drawing nothing.
     *
     * @param ctx the target 2D context.
     * @param view the world->pixel transform.
     * @param pose the robot pose `{x, y, h}` (cm, cm, deg).
     * @param visionCfg the robot-local cone geometry.
     * @param project the projection fn (pass `CTPS_OBJ.project`).
     */
    function drawVisionCone(ctx, view, pose, visionCfg, project) {
        if (!pose || !visionCfg || !project || !_localToWorld) {
            return;
        }
        var tokens = _theTokens();
        var reach = (visionCfg.range_cm === null || visionCfg.range_cm === undefined ||
                     !(visionCfg.range_cm > 0))
            ? CONE_UNBOUNDED_CM : visionCfg.range_cm;
        var half = visionCfg.half_angle_deg * DEG;
        var ox = visionCfg.origin_x_cm;
        var oy = visionCfg.origin_y_cm;

        // Apex + the two edge-ray tips, all in robot-local (+x forward), mapped
        // to world through the pose so the wedge rotates with the heading.
        var apexLocal = { x: ox, y: oy };
        var leftLocal = {
            x: ox + reach * Math.cos(half),
            y: oy + reach * Math.sin(half),
        };
        var rightLocal = {
            x: ox + reach * Math.cos(-half),
            y: oy + reach * Math.sin(-half),
        };
        var apex = _localToWorld(pose, apexLocal.x, apexLocal.y);
        var left = _localToWorld(pose, leftLocal.x, leftLocal.y);
        var right = _localToWorld(pose, rightLocal.x, rightLocal.y);
        var pa = project(view, apex.x, apex.y);
        var pl = project(view, left.x, left.y);
        var pr = project(view, right.x, right.y);

        ctx.save();
        ctx.beginPath();
        ctx.moveTo(pa.px, pa.py);
        ctx.lineTo(pl.px, pl.py);
        ctx.lineTo(pr.px, pr.py);
        ctx.closePath();
        ctx.globalAlpha = CONE_FILL_ALPHA;
        ctx.fillStyle = tokens.accent || "";
        ctx.fill();
        ctx.globalAlpha = 1;
        ctx.lineWidth = CONE_STROKE_W;
        ctx.strokeStyle = tokens.accent || "";
        ctx.stroke();
        ctx.restore();
    }

    // ----- 3. embed resolution (emitted-page globals + upload leg) ---------

    /** The emitted `MOSAIC` global, or null (mosaic-free page). */
    function _embeddedMosaic() {
        return (typeof MOSAIC !== "undefined") ? MOSAIC : null;
    }

    /** The emitted `MOSAIC_META` global, or null. */
    function _embeddedMeta() {
        return (typeof MOSAIC_META !== "undefined") ? MOSAIC_META : null;
    }

    /** The emitted `OBJECTS` layout global, or null. */
    function _embeddedObjects() {
        return (typeof OBJECTS !== "undefined") ? OBJECTS : null;
    }

    /** The cone from a loaded `robot.vision` block (finite fields only), or null. */
    function _coneFromRobot(robot) {
        if (!robot || typeof robot !== "object") {
            return null;
        }
        var v = robot.vision;
        if (!v || typeof v !== "object") {
            return null;
        }
        if (typeof v.origin_x_cm !== "number" || typeof v.origin_y_cm !== "number" ||
            typeof v.half_angle_deg !== "number") {
            return null;
        }
        var range = (v.range_cm === null || v.range_cm === undefined) ? null : v.range_cm;
        return {
            origin_x_cm: v.origin_x_cm,
            origin_y_cm: v.origin_y_cm,
            half_angle_deg: v.half_angle_deg,
            range_cm: range,
        };
    }

    /**
     * Derive a vision embed (`{cells, meta, cone}`) from an UPLOADED layout dict
     * (ADR-056 upload leg): resolve the layout's own `mosaic` geometry block to
     * world cell CENTRES via `mosaicCellCenters` (positions only — an uploaded
     * mosaic carries geometry + optional pinned colours, but the player never
     * synthesizes a roll, so uploaded cells render as the no-roll grid outline),
     * and lift the cone from `robot.vision`. Returns null when the dict carries
     * neither, so `store.loadLayout` leaves the current embed in place.
     *
     * Called by store.js on every `loadLayout` (the upload path); the
     * EMITTED-page path uses the pre-resolved `MOSAIC` global instead (seated by
     * this module's init), never this function.
     *
     * @param layoutDict an uploaded `layout.json`-shaped dict.
     * @return `{cells, meta, cone}` (any field may be null), or null when the
     *     dict has no mosaic geometry and no cone.
     */
    function embedFromLayout(layoutDict) {
        if (!layoutDict || typeof layoutDict !== "object") {
            return null;
        }
        var cone = _coneFromRobot(layoutDict.robot);
        var geom = layoutDict.mosaic;
        var cells = null;
        if (geom && typeof geom === "object") {
            var centers = mosaicCellCenters(geom);
            if (centers.length === CELL_COUNT) {
                cells = centers; // positions only — no `color` => no-roll outline
            }
        }
        if (!cells && !cone) {
            return null;
        }
        return { cells: cells, meta: _embeddedMeta(), cone: cone };
    }

    // ----- 4. canvas layers (trail canvas, like the cargo layers) ----------

    /** The store's resolved mosaic cells (embed or upload leg), or null. */
    function _stateCells(state) {
        return (state && state.mosaicCells) || null;
    }

    /** The store's colour vocabulary, or null. */
    function _stateMeta(state) {
        return (state && state.mosaicMeta) || null;
    }

    /** The store's robot-local vision cone, or null. */
    function _stateCone(state) {
        return (state && state.visionCone) || null;
    }

    /**
     * The cell side in cm for the draw. Prefers the embedded MOSAIC's own
     * geometry (`MOSAIC.cell_w_cm` is not carried, so fall back to the distance
     * between two same-row cells' centres is overkill) — the emitted cells carry
     * only centres, so this uses a small default that reads at field scale.
     * Kept as a single named helper so the mosaic layer + editor share one size
     * policy if extended.
     */
    function _cellSideCm() {
        // The emitted MOSAIC.cells carry centres only (resolve_cells drops the
        // cell size); a small square at each centre reads clearly at field scale.
        return 5;
    }

    /**
     * mosaic layer (trail canvas): the 12 world mosaic cells at their resolved
     * positions, coloured when a roll is present, else outlined. Static in world
     * space (the cells do not move with the robot) — but it rides the trail
     * canvas (cleared+redrawn each repaint) so a cone/robot repaint keeps it
     * painted. No-op when no mosaic is present.
     */
    function _drawMosaicLayer(ctx, view, state) {
        var cells = _stateCells(state);
        if (!cells || cells.length === 0) {
            return;
        }
        drawMosaicCells(ctx, view, cells, OBJ.project, _stateMeta(state), _cellSideCm());
    }

    /**
     * vision-cone layer (trail canvas): the robot-attached cone at the
     * INTERPOLATED pose for the current playhead (`CTPS.engine.poseAt(state.t)`)
     * — a per-frame overlay that moves with the robot, NOT the resolve-once cargo
     * scene. No-op when no cone is configured.
     */
    function _drawConeLayer(ctx, view, state) {
        var cone = _stateCone(state);
        if (!cone) {
            return;
        }
        var pose = CTPS.engine.poseAt(state.t);
        drawVisionCone(ctx, view, pose, cone, OBJ.project);
    }

    CTPS.layers.register({
        id: "mosaic",
        canvasId: "trail",
        visible: function (state) {
            return !!(state.layers && state.layers.mosaic);
        },
        draw: _drawMosaicLayer,
        invalidateOn: ["mosaicCells", "mosaicMeta", "layers"],
    });
    CTPS.layers.register({
        id: "vision-cone",
        canvasId: "trail",
        visible: function (state) {
            return !!(state.layers && state.layers["vision-cone"]);
        },
        draw: _drawConeLayer,
        invalidateOn: ["t", "visionCone", "layers"],
    });

    // ----- 5. per-frame read HUD (#ctps-vision — the VISIBLE panel) --------

    /** Query the `#ctps-vision` panel's driven elements (may be absent). */
    function _queryVisionDom() {
        return {
            panel: document.getElementById("ctps-vision"),
            hud: document.getElementById("ctps-vision-hud"),
            count: document.getElementById("ctps-vision-hud-count"),
            detail: document.getElementById("ctps-vision-hud-detail"),
            badges: document.getElementById("ctps-vision-badges"),
        };
    }

    /** Count the true entries of a boolean array. */
    function _countTrue(bools) {
        var k = 0;
        for (var i = 0; i < bools.length; i++) {
            if (bools[i]) {
                k++;
            }
        }
        return k;
    }

    /** The index of the FIRST cell not visible, or -1 when all are visible. */
    function _firstHidden(bools) {
        for (var i = 0; i < bools.length; i++) {
            if (!bools[i]) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Append one badge chip (`.ctps-vision-badge`) with `text` to `badgesEl`.
     * Text + class only — never an inline style (the ADR-053 carve-out).
     */
    function _appendBadge(badgesEl, text) {
        var badge = document.createElement("span");
        badge.className = "ctps-vision-badge";
        badge.textContent = text;
        badgesEl.appendChild(badge);
    }

    /**
     * Rebuild the `#ctps-vision-badges` row for the current state: a "no mosaic
     * to read" badge when no cells are present, a "no roll recorded" badge when
     * the embedded mosaic carried geometry but no roll (`MOSAIC.roll === false`),
     * a "geometry changed since recording" badge when the trace's geometry hash
     * no longer matches (`MOSAIC.geom_match === false`), and a "no vision cone
     * configured" badge when the cone is absent. Cleared + rebuilt each call.
     */
    function _renderBadges(dom, state) {
        var badgesEl = dom.badges;
        if (!badgesEl) {
            return;
        }
        while (badgesEl.firstChild) {
            badgesEl.removeChild(badgesEl.firstChild);
        }
        var cells = _stateCells(state);
        var mosaic = _embeddedMosaic();
        if (!cells || cells.length === 0) {
            _appendBadge(badgesEl, "no mosaic to read");
        } else {
            // Roll / geometry flags come from the EMITTED MOSAIC global (an
            // uploaded mosaic has no roll and no recorded geometry hash, so it is
            // treated as the no-roll grid — matched by the first cell lacking a
            // `color`).
            var rolled = mosaic ? (mosaic.roll !== false) : _cellsHaveColor(cells);
            if (!rolled) {
                _appendBadge(badgesEl, "no roll recorded");
            }
            if (mosaic && mosaic.geom_match === false) {
                _appendBadge(badgesEl, "geometry changed since recording");
            }
        }
        if (!_stateCone(state)) {
            _appendBadge(badgesEl, "no vision cone configured");
        }
    }

    /** True when the first cell carries a `color` (a roll is present). */
    function _cellsHaveColor(cells) {
        return !!(cells && cells.length > 0 && cells[0].color);
    }

    /**
     * Update the read HUD (`.ctps-vision-hud-count` / `-detail`) + the panel's
     * success modifier for the current playhead. When both a cone and cells are
     * present, compute `visibleCells(poseAt(t), cone, cells)`, show `k/12`, name
     * the first cell outside the cone in the detail when `k < 12`, and add the
     * `ctps-vision-hud--ok` modifier on the count at 12/12 (the green success
     * state). With no cone or no cells, show an em-dash count and clear the
     * detail (the badges explain why).
     */
    function _renderHud(dom, state) {
        var countEl = dom.count;
        var detailEl = dom.detail;
        var cells = _stateCells(state);
        var cone = _stateCone(state);

        if (!cone || !cells || cells.length === 0) {
            if (countEl) {
                countEl.textContent = "—"; // em dash — nothing to read
                countEl.classList.remove("ctps-vision-hud--ok");
            }
            if (detailEl) {
                detailEl.textContent = "";
            }
            return;
        }

        var pose = CTPS.engine.poseAt(state.t);
        var bools = visibleCells(pose, cone, cells);
        var total = bools.length;
        var k = _countTrue(bools);

        if (countEl) {
            countEl.textContent = k + "/" + total;
            countEl.classList.toggle("ctps-vision-hud--ok",
                readMosaicWouldSucceed(bools));
        }
        if (detailEl) {
            if (k < total) {
                var hidden = _firstHidden(bools);
                var cellIndex = (cells[hidden] && typeof cells[hidden].index === "number")
                    ? cells[hidden].index : hidden;
                detailEl.textContent = "cell " + cellIndex + " outside cone";
            } else {
                detailEl.textContent = "all cells in cone";
            }
        }
    }

    /** Rebuild every state-driven piece of the vision panel in one pass. */
    function _renderVision(dom, state) {
        if (!dom.panel) {
            return; // no #ctps-vision panel (a legacy page) — nothing to drive
        }
        _renderHud(dom, state);
        _renderBadges(dom, state);
    }

    // ----- boot ------------------------------------------------------------

    /**
     * Seat the EMITTED-page vision embed onto the store: the pre-resolved
     * `MOSAIC` cells (Python-authoritative geometry x the trace roll — carrying
     * `color` per cell when a roll exists), the `MOSAIC_META` colour vocabulary,
     * and the cone from the loaded `robot.vision`. Guards every global (a
     * mosaic-free page leaves them undefined, so this seats nulls and the
     * layers/HUD no-op).
     *
     * PRECEDENCE: this runs in `_initVision`, which is inited AFTER the cargo
     * bootstrap's `store.loadLayout(OBJECTS)` (cargo.js is inlined earlier). On
     * an emitted page the layout global carries the same `mosaic` geometry, so
     * `loadLayout` first seats the upload-leg embed (positions-only, no colour);
     * this call THEN overwrites it with the pre-resolved `MOSAIC` cells, so the
     * recorded roll colours win. The two seats are idempotent (same positions);
     * only the colour differs, and the pre-resolved one is authoritative.
     */
    function _seatEmbeddedVision(store) {
        var mosaic = _embeddedMosaic();
        var meta = _embeddedMeta();
        var objects = _embeddedObjects();
        var cone = _coneFromRobot(objects && objects.robot);
        var cells = (mosaic && mosaic.cells && mosaic.cells.length) ? mosaic.cells : null;
        if (cells || meta || cone) {
            store.setVisionEmbed(cells, meta, cone);
        }
    }

    /**
     * Wire the vision panel: seat the embedded mosaic/cone, paint the panel once
     * from whatever the store holds, then subscribe so every later playhead move
     * (`t`) or embed change (an upload) repaints the HUD/badges. The layers are
     * repainted by the renderer off their own `invalidateOn`, so this init only
     * owns the DOM HUD.
     */
    function _initVision(ctx) {
        var store = (ctx && ctx.store) || CTPS.store;
        if (!store) {
            return;
        }
        _seatEmbeddedVision(store);
        var dom = _queryVisionDom();
        _renderVision(dom, store);
        store.subscribe(function (changedKeys, state) {
            if (changedKeys.indexOf("t") !== -1 ||
                changedKeys.indexOf("mosaicCells") !== -1 ||
                changedKeys.indexOf("mosaicMeta") !== -1 ||
                changedKeys.indexOf("visionCone") !== -1) {
                _renderVision(dom, state);
            }
        });
    }

    // ----- public surface --------------------------------------------------
    // Extend (never replace) the T0 namespace so an early editor reference still
    // finds the object; `embedFromLayout` is reached by store.js at call time.
    window.CTPS_VISION = window.CTPS_VISION || {};
    window.CTPS_VISION.visibleCells = visibleCells;
    window.CTPS_VISION.readMosaicWouldSucceed = readMosaicWouldSucceed;
    window.CTPS_VISION.mosaicCellCenters = mosaicCellCenters;
    window.CTPS_VISION.drawVisionCone = drawVisionCone;
    window.CTPS_VISION.drawMosaicCells = drawMosaicCells;

    // Also exposed on CTPS.vision so store.js's upload-leg loadLayout reaches
    // `embedFromLayout` at call time (store.js is inlined before this file).
    CTPS.vision = CTPS.vision || {};
    CTPS.vision.embedFromLayout = embedFromLayout;
    CTPS.vision.visibleCells = visibleCells;
    CTPS.vision.readMosaicWouldSucceed = readMosaicWouldSucceed;
    CTPS.vision.mosaicCellCenters = mosaicCellCenters;

    CTPS._inits.push(_initVision);
})();
