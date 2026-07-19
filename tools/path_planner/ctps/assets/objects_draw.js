/* CTPS — Clear Trajectory Prediction Simulator: field-object + zone canvas draws.
 *
 * Transform-agnostic canvas draws for the manipulator cargo model (object/cargo
 * plan, T5 -- issue #57), exposed on the shared `window.CTPS_OBJ` global so both
 * the player (cargo.js's objects/zones layers) and the offline layout editor
 * (T8) can call them against their OWN world->pixel transform:
 *
 *   - `project(view, x, y)` — the world(cm)->pixel formula. This is a DELIBERATE
 *     DUPLICATE of renderer.js's private `_project` (Contracts -> Projection):
 *     renderer.js is byte-frozen next to the pinned seek core and is NOT
 *     refactored or given an export, so the ~single-line transform is copied
 *     here instead. The player passes renderer's own `view`; the editor passes
 *     its own computed `view` — both flow through the same pure formula.
 *   - `drawObject(ctx, view, obj, project, opts)` — one field object, styled by
 *     TYPE (tile = filled authored color; cargo = hollow --ctps-hold outline;
 *     fixed = faint dashed --ctps-faint) and pathed by SHAPE (rect, rotated by
 *     `heading_deg`; circle). `opts.x`/`opts.y` optionally OVERRIDE the object's
 *     authored world centre (the player draws dropped/dumped objects at their
 *     resolved `placed` spot).
 *   - `drawZones(ctx, view, pose, robotCfg, project)` — the robot-local
 *     manipulator overlay (grab zone, rear-box zone, per-lane drop points),
 *     mapped to world through the robot pose (`world = pose + Rot(h)*local`).
 *
 * Token colours (--ctps-hold / --ctps-faint) are resolved once from the
 * stylesheet and cached (they are static in this app) — the same
 * getComputedStyle channel renderer.js uses, never a hard-coded hex. Object
 * fill uses the object's own authored `color` hex directly (a canvas
 * `fillStyle`, not a DOM inline style). No DOM `.style` writes, no store access,
 * no network. Vanilla ES5 — var/function only, works offline.
 *
 * Coordinate convention (matches renderer.js / player.js): centimetres, degrees
 * in [0, 360), 0 deg = +x (East), CCW positive, world +Y North. Canvas y grows
 * downward, so the transform flips Y and a rect's draw angle is the negated
 * `heading_deg`.
 */

"use strict";

(function () {
    // ----- paint constants (named, no magic literals inline) ---------------
    var OBJ_STROKE_W = 1.5;      // cargo/fixed outline stroke width (px)
    var OBJ_DASH = [4, 3];       // fixed-object dashed outline pattern (px)
    var ZONE_ALPHA = 0.9;        // manipulator-zone overlay opacity
    var ZONE_STROKE_W = 2;       // zone outline stroke width (px)
    var ZONE_DASH = [3, 3];      // zone outline dashed pattern (px)
    var ZONE_DOT_R = 3;          // drop-point marker radius (px)
    var DEPLOY_FILL_ALPHA = 0.22; // translucent fill for a deployed area
    var DEPLOY_STROKE_W = 2;      // deployed-area outline width (px) — matches ZONE_STROKE_W
    var DEPLOY_DOT_R = 4;         // deployed drop-point marker radius (px)

    // Object types + shapes — exact strings (Contracts: Naming; mirror of
    // ctps/layout.py's SHAPE_*/OBJECT_TYPE_* constants).
    var TYPE_CARGO = "cargo";
    var TYPE_FIXED = "fixed";
    var SHAPE_CIRCLE = "circle";

    // ----- token colour resolution (cached; static in this app) ------------
    // Resolved lazily on the first draw (the inlined <style> is parsed before
    // any layer paints) and cached, mirroring renderer.js's _resolveColors
    // discipline so a per-frame zones redraw never re-hits getComputedStyle.
    var _tokens = null;

    /**
     * Read one CSS custom property off the document root, trimmed. Returns ""
     * when unset so a canvas colour assignment is simply ignored (the
     * degenerate no-stylesheet case), never a hard-coded hex fallback.
     */
    function _cssVar(name) {
        var v = getComputedStyle(document.documentElement).getPropertyValue(name);
        return v ? v.replace(/^\s+|\s+$/g, "") : "";
    }

    /** Resolve + cache the object/zone token colours from the stylesheet. */
    function _resolveTokens() {
        _tokens = {
            hold: _cssVar("--ctps-hold"),
            faint: _cssVar("--ctps-faint"),
            accent: _cssVar("--ctps-accent"),
        };
    }

    /** The cached token table, resolving it on first use. */
    function _theTokens() {
        if (!_tokens) {
            _resolveTokens();
        }
        return _tokens;
    }

    // ----- world<->pixel transform (DUPLICATE of renderer.js _project) -----

    /**
     * Project a world point (cm) to canvas pixels through `view`. Uniform
     * scale, centred within the padded box, Y flipped (world +Y up -> canvas y
     * down). A verbatim duplicate of renderer.js's private `_project` — the
     * transform is copied, not shared, so renderer.js stays untouched
     * (Contracts -> Projection).
     *
     * @param view the renderer's (or editor's) computed transform.
     * @param x world X in centimetres.
     * @param y world Y in centimetres.
     * @return `{ px, py }` canvas-pixel coordinates.
     */
    function project(view, x, y) {
        return {
            px: view.padX + (x - view.x0) * view.scale,
            py: view.cssH - view.padY - (y - view.y0) * view.scale,
        };
    }

    // ----- small geometry helpers ------------------------------------------

    /**
     * Map a robot-local point (cm) to world (cm) at pose `{x, y, h}`:
     * `world = pose.xy + Rot(h) . local.xy`, h in degrees CCW. Robot-local
     * frame: origin = robot centre, +x = heading-forward, +y = robot-left.
     */
    function _localToWorld(pose, lx, ly) {
        var rad = pose.h * Math.PI / 180;
        var c = Math.cos(rad);
        var s = Math.sin(rad);
        return { x: pose.x + lx * c - ly * s, y: pose.y + lx * s + ly * c };
    }

    /**
     * Read one offset component, defaulting to 0 when missing or not a
     * finite number (defensive, like the rest of this file's readers).
     */
    function _finiteOr0(v) {
        return (typeof v === "number" && isFinite(v)) ? v : 0;
    }

    /**
     * Shift an axle pose to the body-centre pose using the layout's
     * `robot.axle_offset` (robot-local `{forward_cm, lateral_cm}`; each
     * defaults to 0 when the block or key is missing/not a finite number).
     * When both components are 0, returns `pose` UNCHANGED — the SAME
     * object, no allocation — so a no-offset render stays byte-identical to
     * today. Otherwise returns a SHALLOW COPY of `pose` with `x`/`y` moved
     * to the body centre and `h` (and any other keys) left untouched. The
     * body centre reuses `_localToWorld` (Contracts: single-source the
     * offset math, do not re-derive the rotation) with `local = (forward_cm,
     * lateral_cm)`. Never throws: a null/malformed `pose` is returned
     * unchanged, matching this file's defensive/non-fatal style (e.g.
     * `drawZones`' `!robotCfg || !pose` no-op).
     *
     * @param pose the axle pose `{ x, y, h, ... }` (cm, cm, deg).
     * @param robotCfg the layout `robot` section (reads `.axle_offset`).
     * @return the body-centre pose (the SAME object when there is no offset).
     */
    function axleToBodyPose(pose, robotCfg) {
        if (!pose) {
            return pose;
        }
        var offset = robotCfg && robotCfg.axle_offset;
        var fwd = offset ? _finiteOr0(offset.forward_cm) : 0;
        var lat = offset ? _finiteOr0(offset.lateral_cm) : 0;
        if (fwd === 0 && lat === 0) {
            return pose;
        }
        var out = {};
        for (var key in pose) {
            if (Object.prototype.hasOwnProperty.call(pose, key)) {
                out[key] = pose[key];
            }
        }
        var w = _localToWorld(pose, fwd, lat);
        out.x = w.x;
        out.y = w.y;
        return out;
    }

    /** Begin an axis-aligned rectangle path (in the CURRENT ctx transform). */
    function _pathRect(ctx, x, y, w, h) {
        ctx.beginPath();
        ctx.rect(x, y, w, h);
    }

    /** Begin a circle path centred at (cx, cy) with pixel radius r. */
    function _pathCircle(ctx, cx, cy, r) {
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
    }

    // ----- object drawing ---------------------------------------------------

    /**
     * Apply the type-specific fill/stroke to the CURRENT path: a tile is filled
     * with its authored hex; a cargo is a hollow --ctps-hold outline; a fixed
     * object is a faint dashed --ctps-faint outline. The line dash is scoped by
     * the caller's save/restore, so a non-dashed path is unaffected.
     *
     * @param ctx the 2D context (path already built, transform already set).
     * @param type the object type ("tile" | "cargo" | "fixed").
     * @param color the object's authored `#RRGGBB` hex (tile fill).
     */
    function _applyTypeStyle(ctx, type, color) {
        var tokens = _theTokens();
        if (type === TYPE_CARGO) {
            ctx.lineWidth = OBJ_STROKE_W;
            ctx.strokeStyle = tokens.hold || color;
            ctx.stroke();
        } else if (type === TYPE_FIXED) {
            ctx.lineWidth = OBJ_STROKE_W;
            ctx.strokeStyle = tokens.faint || color;
            if (ctx.setLineDash) {
                ctx.setLineDash(OBJ_DASH);
            }
            ctx.stroke();
        } else {
            // tile (default): solid fill in the authored colour.
            ctx.fillStyle = color;
            ctx.fill();
        }
    }

    /**
     * Draw one field object through `project`. The rect path is rotated by the
     * object's `heading_deg` (negated for the Y-flipped canvas); the circle path
     * is rotation-invariant. `opts.x`/`opts.y`, when finite numbers, OVERRIDE the
     * object's authored `x_cm`/`y_cm` world centre — the player passes a resolved
     * `placed` spot for dropped/dumped objects; the editor omits them to draw at
     * the authored position.
     *
     * @param ctx the target 2D context.
     * @param view the world->pixel transform (renderer's or editor's).
     * @param obj a layout object `{ id, shape, type, color, x_cm, y_cm,
     *     heading_deg, w_cm?, h_cm?, radius_cm? }`.
     * @param project the projection fn (pass `CTPS_OBJ.project`).
     * @param opts optional `{ x, y }` world-centre override.
     */
    function drawObject(ctx, view, obj, project, opts) {
        if (!obj) {
            return;
        }
        opts = opts || {};
        var cx = (typeof opts.x === "number") ? opts.x : obj.x_cm;
        var cy = (typeof opts.y === "number") ? opts.y : obj.y_cm;
        var centre = project(view, cx, cy);
        var scale = view.scale;
        ctx.save();
        if (obj.shape === SHAPE_CIRCLE) {
            var radiusCm = (typeof obj.radius_cm === "number") ? obj.radius_cm : 0;
            _pathCircle(ctx, centre.px, centre.py, radiusCm * scale);
        } else {
            var wPx = ((typeof obj.w_cm === "number") ? obj.w_cm : 0) * scale;
            var hPx = ((typeof obj.h_cm === "number") ? obj.h_cm : 0) * scale;
            var ang = -((typeof obj.heading_deg === "number") ? obj.heading_deg : 0) *
                Math.PI / 180;
            ctx.translate(centre.px, centre.py);
            ctx.rotate(ang);
            _pathRect(ctx, -wPx / 2, -hPx / 2, wPx, hPx);
        }
        _applyTypeStyle(ctx, obj.type, obj.color);
        ctx.restore();
    }

    // ----- zone overlay drawing --------------------------------------------

    /**
     * Build (but do not stroke/fill) one robot-local rect zone's path as a
     * world polygon: its four local corners are mapped to world through the
     * pose, then projected and joined. The zone is axis-aligned in
     * robot-local coordinates, so it renders as a rotated rectangle that
     * follows the robot heading. Callers apply their own fill/stroke after
     * this returns — this helper ONLY builds the current ctx path.
     */
    function _pathZoneRectWorld(ctx, view, pose, zone, project) {
        if (!zone) {
            return;
        }
        var hw = zone.w_cm / 2;
        var hh = zone.h_cm / 2;
        var corners = [
            [zone.x_cm - hw, zone.y_cm - hh],
            [zone.x_cm + hw, zone.y_cm - hh],
            [zone.x_cm + hw, zone.y_cm + hh],
            [zone.x_cm - hw, zone.y_cm + hh],
        ];
        ctx.beginPath();
        for (var i = 0; i < corners.length; i++) {
            var w = _localToWorld(pose, corners[i][0], corners[i][1]);
            var p = project(view, w.x, w.y);
            if (i === 0) {
                ctx.moveTo(p.px, p.py);
            } else {
                ctx.lineTo(p.px, p.py);
            }
        }
        ctx.closePath();
    }

    /**
     * Stroke one robot-local rect zone as a world polygon. Thin wrapper over
     * `_pathZoneRectWorld` (path-building only) + `ctx.stroke()` — kept as a
     * separate function so the static `zones` layer's stroked-only output
     * stays byte-identical to before the extraction.
     */
    function _drawZoneRect(ctx, view, pose, zone, project) {
        if (!zone) {
            return;
        }
        _pathZoneRectWorld(ctx, view, pose, zone, project);
        ctx.stroke();
    }

    /**
     * Draw the faint robot-local manipulator overlay at pose `{x, y, h}`: the
     * grab zone, the rear-box zone, and the per-lane drop points, all mapped to
     * world and stroked in --ctps-hold. A missing `robotCfg`/`pose` degrades to
     * drawing nothing (the resolver already treats those cases as no-ops).
     *
     * @param ctx the target 2D context.
     * @param view the world->pixel transform.
     * @param pose the robot pose `{ x, y, h }` (cm, cm, deg).
     * @param robotCfg the layout `robot` section (`grab_zone`/`box_zone`/
     *     `drop_points`).
     * @param project the projection fn (pass `CTPS_OBJ.project`).
     */
    function drawZones(ctx, view, pose, robotCfg, project) {
        if (!robotCfg || !pose) {
            return;
        }
        var tokens = _theTokens();
        ctx.save();
        ctx.globalAlpha = ZONE_ALPHA;
        ctx.lineWidth = ZONE_STROKE_W;
        ctx.strokeStyle = tokens.hold || "";
        if (ctx.setLineDash) {
            ctx.setLineDash(ZONE_DASH);
        }
        _drawZoneRect(ctx, view, pose, robotCfg.grab_zone, project);
        _drawZoneRect(ctx, view, pose, robotCfg.box_zone, project);
        // Drop points: small solid rings (no dash) at each lane's world point.
        if (ctx.setLineDash) {
            ctx.setLineDash([]);
        }
        var drops = robotCfg.drop_points || [];
        for (var i = 0; i < drops.length; i++) {
            var dp = drops[i];
            if (!dp) {
                continue;
            }
            var w = _localToWorld(pose, dp.x_cm, dp.y_cm);
            var p = project(view, w.x, w.y);
            _pathCircle(ctx, p.px, p.py, ZONE_DOT_R);
            ctx.stroke();
        }
        ctx.restore();
    }

    /**
     * Draw ONLY the manipulator areas whose `mask` flag is true, in the
     * filled-accent "deployed" style (translucent --ctps-accent fill + solid
     * --ctps-accent outline; drop-points as filled accent dots) — visually
     * distinct from `drawZones`' always-on dashed --ctps-hold overlay. A
     * missing `robotCfg`/`pose`/`mask`, or an absent zone for a true mask
     * flag, degrades to drawing nothing for that area.
     *
     * @param ctx the target 2D context.
     * @param view the world->pixel transform.
     * @param pose the robot pose `{ x, y, h }` (cm, cm, deg).
     * @param robotCfg the layout `robot` section (`grab_zone`/`box_zone`/
     *     `drop_points`).
     * @param project the projection fn (pass `CTPS_OBJ.project`).
     * @param mask `{ grab, box, dump }` — which areas are currently deployed.
     */
    function drawDeployedZones(ctx, view, pose, robotCfg, project, mask) {
        if (!robotCfg || !pose || !mask) {
            return;
        }
        var tokens = _theTokens();
        ctx.save();
        ctx.strokeStyle = tokens.accent || "";
        ctx.fillStyle = tokens.accent || "";
        ctx.lineWidth = DEPLOY_STROKE_W;
        if (mask.grab && robotCfg.grab_zone) {
            _pathZoneRectWorld(ctx, view, pose, robotCfg.grab_zone, project);
            ctx.globalAlpha = DEPLOY_FILL_ALPHA;
            ctx.fill();
            ctx.globalAlpha = 1;
            ctx.stroke();
        }
        if (mask.box && robotCfg.box_zone) {
            _pathZoneRectWorld(ctx, view, pose, robotCfg.box_zone, project);
            ctx.globalAlpha = DEPLOY_FILL_ALPHA;
            ctx.fill();
            ctx.globalAlpha = 1;
            ctx.stroke();
        }
        if (mask.dump) {
            var drops = robotCfg.drop_points || [];
            for (var i = 0; i < drops.length; i++) {
                var dp = drops[i];
                if (!dp) {
                    continue;
                }
                var w = _localToWorld(pose, dp.x_cm, dp.y_cm);
                var p = project(view, w.x, w.y);
                _pathCircle(ctx, p.px, p.py, DEPLOY_DOT_R);
                ctx.fill();
            }
        }
        ctx.restore();
    }

    // ----- public surface ---------------------------------------------------
    window.CTPS_OBJ = window.CTPS_OBJ || {};
    window.CTPS_OBJ.project = project;
    window.CTPS_OBJ.drawObject = drawObject;
    window.CTPS_OBJ.drawZones = drawZones;
    window.CTPS_OBJ.drawDeployedZones = drawDeployedZones;
    window.CTPS_OBJ.localToWorld = _localToWorld;
    window.CTPS_OBJ.axleToBodyPose = axleToBodyPose;
})();
