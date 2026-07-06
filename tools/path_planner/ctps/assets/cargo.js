/* CTPS — Clear Trajectory Prediction Simulator: cargo resolver + layers.
 *
 * The manipulator cargo model for the emitted player (object/cargo plan --
 * issue #57). This file has two clearly separated sections:
 *
 *   T5 (this section) — RESOLVER + LAYERS + STORE-WIRING:
 *     - `cargo_resolve(track, objects, robotCfg)` — the resolve-once,
 *       event-indexed scene timeline. One snapshot per `manip` event, in event
 *       order, computed in a single pure pass (events x poseAt(t) x zone
 *       geometry). Scrubbing is then a pure bisect (see store.js), never an
 *       incremental fold-on-scrub — so a backward seek shows the exact placement
 *       with no stale state (AC10). Exposed on `CTPS.cargo` so store.js's
 *       `loadLayout` can run it (store.js is inlined BEFORE this file, so it
 *       resolves `CTPS.cargo.resolve` at call time, not IIFE time).
 *     - The `objects` and `zones` canvas layers, registered on the `trail`
 *       canvas (below the actors/robot canvas). Both redraw by clear+redraw on
 *       each trail repaint (the renderer clears the whole canvas per repaint) --
 *       NEVER a getImageData/putImageData snapshot, which works in device px and
 *       ghosts on devicePixelRatio > 1.
 *     - The embedded-layout bootstrap: an init that, when an `OBJECTS` layout
 *       constant is embedded at emit time, hands it to `store.loadLayout`.
 *
 *   T6 (the second section, currently an empty labelled stub) — the two-tab
 *     cargo PANEL, the `#ctps-layout-file` upload wiring, the `#ctps-hud-holding`
 *     HUD line, and the Holding timeline lane. NOT implemented here.
 *
 * Reads each event's structured `method`/`arg` (track_builder.py) — never the
 * display `label`. Consumes the pinned engine via `CTPS.engine` (poseAt) and the
 * transform-agnostic draws via `CTPS_OBJ` (both inlined before this file). No DOM
 * `.style` writes in this section. Vanilla ES5 — var/function only, works
 * offline.
 */

"use strict";

(function () {
    var CTPS = window.CTPS;
    var OBJ = window.CTPS_OBJ;

    // ===================================================================
    // T5 — RESOLVER + LAYERS + STORE-WIRING
    // ===================================================================

    // ----- mirrored player constants (single-sourced in ctps/config.py) ----
    // Magazine geometry + rear-box capacity are fixed player constants, NOT
    // authored per-layout — mirrored here from config.py (MAGAZINE_LANES /
    // MAGAZINE_DEPTH / REAR_BOX_CAP) so a layout author can never desync the
    // store shape from what the panel renders.
    var MAGAZINE_LANES = 4;
    var MAGAZINE_DEPTH = 3;
    var REAR_BOX_CAP = 1;

    // Trace manip method values — exact strings (Contracts: Naming; recorder
    // <-> resolver). `reset` intentionally changes no cargo state.
    var METHOD_GRAB = "grab";
    var METHOD_RELEASE = "release";
    var METHOD_SORT_TO = "sortTo";
    var METHOD_DUMP = "dump";
    var METHOD_BOX_CLOSE = "boxClose";
    var METHOD_BOX_OPEN = "boxOpen";

    var TYPE_TILE = "tile";
    var TYPE_CARGO = "cargo";

    // ----- working-state helpers -------------------------------------------

    /** A fresh mutable working state (in-hand empty, all lanes empty, box empty). */
    function _emptyState() {
        var magazine = [];
        for (var lane = 0; lane < MAGAZINE_LANES; lane++) {
            magazine.push([]);
        }
        return { inHand: null, magazine: magazine, rearBox: null, placed: {} };
    }

    /**
     * Freeze the mutable state into an immutable per-event snapshot (deep-copied
     * lanes + placed map) tagged with its source event index and time.
     */
    function _snapshotState(state, eventIndex, t) {
        var magazine = [];
        for (var lane = 0; lane < state.magazine.length; lane++) {
            magazine.push(state.magazine[lane].slice());
        }
        var placed = {};
        for (var id in state.placed) {
            if (Object.prototype.hasOwnProperty.call(state.placed, id)) {
                placed[id] = { x_cm: state.placed[id].x_cm, y_cm: state.placed[id].y_cm };
            }
        }
        return {
            eventIndex: eventIndex,
            t: t,
            inHand: state.inHand,
            magazine: magazine,
            rearBox: state.rearBox,
            placed: placed,
        };
    }

    /**
     * The pristine scene snapshot (nothing grabbed/stored/placed) with the
     * correct lane count. Used by store.js for the "playhead before the first
     * manip event" case, so a consumer always gets a well-formed snapshot.
     */
    function _emptyScene() {
        return _snapshotState(_emptyState(), -1, 0);
    }

    // ----- geometry (robot-local <-> world) --------------------------------

    /**
     * Express a world point (cm) in the robot-local frame at pose `{x, y, h}`:
     * `local = Rot(-h) . (world - pose.xy)`, h in degrees CCW. Local frame:
     * origin = robot centre, +x = heading-forward, +y = robot-left.
     */
    function _worldToLocal(pose, wx, wy) {
        var rad = pose.h * Math.PI / 180;
        var c = Math.cos(rad);
        var s = Math.sin(rad);
        var dx = wx - pose.x;
        var dy = wy - pose.y;
        return { x: dx * c + dy * s, y: -dx * s + dy * c };
    }

    /** Map a robot-local point (cm) to world (cm): `world = pose.xy + Rot(h).local`. */
    function _localToWorld(pose, lx, ly) {
        var rad = pose.h * Math.PI / 180;
        var c = Math.cos(rad);
        var s = Math.sin(rad);
        return { x: pose.x + lx * c - ly * s, y: pose.y + lx * s + ly * c };
    }

    /** True when a robot-local point falls inside the axis-aligned local zone rect. */
    function _pointInZone(local, zone) {
        return Math.abs(local.x - zone.x_cm) <= zone.w_cm / 2 &&
            Math.abs(local.y - zone.y_cm) <= zone.h_cm / 2;
    }

    // ----- object-position helpers -----------------------------------------

    /** True when object `id` is NOT staged in any store (in-hand / magazine / box). */
    function _isOnField(state, id) {
        if (state.inHand === id || state.rearBox === id) {
            return false;
        }
        for (var lane = 0; lane < state.magazine.length; lane++) {
            var stack = state.magazine[lane];
            for (var k = 0; k < stack.length; k++) {
                if (stack[k] === id) {
                    return false;
                }
            }
        }
        return true;
    }

    /** The object's CURRENT world centre — its `placed` spot if moved, else authored. */
    function _currentWorld(state, obj) {
        var p = state.placed[obj.id];
        if (p) {
            return { x: p.x_cm, y: p.y_cm };
        }
        return { x: obj.x_cm, y: obj.y_cm };
    }

    /** Drop object `id` from the `placed` map (it is entering a store slot). */
    function _clearPlaced(state, id) {
        if (Object.prototype.hasOwnProperty.call(state.placed, id)) {
            delete state.placed[id];
        }
    }

    /**
     * The nearest on-field object of `type` whose CURRENT centre falls inside
     * `zone` at `pose` — or null when none qualifies. "Nearest" is measured to
     * the zone centre (a stand-in for the manipulator's reach point), computed in
     * the robot-local frame (rotation preserves distance).
     */
    function _nearestInZone(state, objects, type, pose, zone) {
        var best = null;
        var bestDist = Infinity;
        for (var i = 0; i < objects.length; i++) {
            var obj = objects[i];
            if (!obj || obj.type !== type || !obj.id) {
                continue;
            }
            if (!_isOnField(state, obj.id)) {
                continue;
            }
            var w = _currentWorld(state, obj);
            var local = _worldToLocal(pose, w.x, w.y);
            if (!_pointInZone(local, zone)) {
                continue;
            }
            var ddx = local.x - zone.x_cm;
            var ddy = local.y - zone.y_cm;
            var dist = ddx * ddx + ddy * ddy;
            if (dist < bestDist) {
                bestDist = dist;
                best = obj;
            }
        }
        return best;
    }

    /**
     * Parse a `sortTo` lane argument to an integer, or -1 when it is not a
     * whole number (the caller then no-ops on the out-of-range check).
     */
    function _laneIndex(arg) {
        var n = (typeof arg === "number") ? arg : parseInt(arg, 10);
        if (typeof n !== "number" || isNaN(n) || n !== Math.floor(n)) {
            return -1;
        }
        return n;
    }

    // ----- per-verb state transitions --------------------------------------

    /** grab: pick the nearest on-field Tile whose centre is in the grab zone. */
    function _doGrab(state, pose, objects, robotCfg) {
        if (state.inHand !== null) {
            return; // in-hand occupied -> logged no-op
        }
        var zone = robotCfg && robotCfg.grab_zone;
        if (!zone) {
            return; // no robot zones -> no-op
        }
        var tile = _nearestInZone(state, objects, TYPE_TILE, pose, zone);
        if (!tile) {
            return; // empty-zone grab -> logged no-op
        }
        state.inHand = tile.id;
        _clearPlaced(state, tile.id);
    }

    /** release: drop the in-hand tile back at the grab-zone world centre. */
    function _doRelease(state, pose, robotCfg) {
        if (state.inHand === null) {
            return; // nothing to drop -> no-op
        }
        var zone = robotCfg && robotCfg.grab_zone;
        if (!zone) {
            return; // no zone -> cannot place -> no-op
        }
        var w = _localToWorld(pose, zone.x_cm, zone.y_cm);
        state.placed[state.inHand] = { x_cm: w.x, y_cm: w.y };
        state.inHand = null;
    }

    /** sortTo(n): file the in-hand tile into magazine lane n (front LAST). */
    function _doSortTo(state, arg) {
        if (state.inHand === null) {
            return; // no in-hand -> no-op
        }
        var n = _laneIndex(arg);
        if (n < 0 || n >= MAGAZINE_LANES) {
            return; // bad lane -> no-op
        }
        var lane = state.magazine[n];
        if (lane.length >= MAGAZINE_DEPTH) {
            return; // full lane -> no-op
        }
        lane.push(state.inHand); // front-to-back fill: front tile is the last
        _clearPlaced(state, state.inHand);
        state.inHand = null;
    }

    /** dump: eject the FRONT tile of each non-empty lane to its lane drop point. */
    function _doDump(state, pose, robotCfg) {
        var drops = (robotCfg && robotCfg.drop_points) || [];
        if (drops.length === 0) {
            return; // no drop points -> no-op (never lose a tile)
        }
        for (var lane = 0; lane < state.magazine.length; lane++) {
            var stack = state.magazine[lane];
            if (stack.length === 0) {
                continue; // empty lane -> nothing ejected
            }
            var id = stack.pop(); // front = last element
            var dp = drops[lane];
            var w = dp ? _localToWorld(pose, dp.x_cm, dp.y_cm)
                : { x: pose.x, y: pose.y };
            state.placed[id] = { x_cm: w.x, y_cm: w.y };
        }
    }

    /** boxClose: capture the nearest on-field Cargo whose centre is in the box zone. */
    function _doBoxClose(state, pose, objects, robotCfg) {
        if (state.rearBox !== null) {
            return; // box full (REAR_BOX_CAP == 1, single slot) -> no-op
        }
        var zone = robotCfg && robotCfg.box_zone;
        if (!zone) {
            return; // no zone -> no-op
        }
        var cargo = _nearestInZone(state, objects, TYPE_CARGO, pose, zone);
        if (!cargo) {
            return; // no Cargo centre in zone -> logged no-op
        }
        state.rearBox = cargo.id;
        _clearPlaced(state, cargo.id);
    }

    /** boxOpen: drop the held Cargo back at the box-zone world centre. */
    function _doBoxOpen(state, pose, robotCfg) {
        if (state.rearBox === null) {
            return; // nothing held -> no-op
        }
        var zone = robotCfg && robotCfg.box_zone;
        if (!zone) {
            return; // no zone -> cannot place -> no-op
        }
        var w = _localToWorld(pose, zone.x_cm, zone.y_cm);
        state.placed[state.rearBox] = { x_cm: w.x, y_cm: w.y };
        state.rearBox = null;
    }

    /**
     * Apply one manip verb to the working state at `pose`. Unknown methods (and
     * `reset`, which is log-only) leave the state unchanged.
     */
    function _applyManip(state, method, arg, pose, objects, robotCfg) {
        if (method === METHOD_GRAB) {
            _doGrab(state, pose, objects, robotCfg);
        } else if (method === METHOD_RELEASE) {
            _doRelease(state, pose, robotCfg);
        } else if (method === METHOD_SORT_TO) {
            _doSortTo(state, arg);
        } else if (method === METHOD_DUMP) {
            _doDump(state, pose, robotCfg);
        } else if (method === METHOD_BOX_CLOSE) {
            _doBoxClose(state, pose, objects, robotCfg);
        } else if (method === METHOD_BOX_OPEN) {
            _doBoxOpen(state, pose, robotCfg);
        }
        // METHOD reset / unknown: no cargo change (snapshot mirrors prior state).
    }

    /**
     * Resolve the whole scene timeline in one pure pass: one snapshot per `manip`
     * event, in event order. Each event is applied against the robot pose at its
     * own time (`CTPS.engine.poseAt`), which projects the grab/box zones and drop
     * points to world. No-op verbs still emit a snapshot (mirroring the prior
     * state) so every manip event has a scene index for the store's bisect.
     *
     * @param track the ingested Data-Model-B track (reads `track.events`).
     * @param objects the layout objects array (`store.objects`).
     * @param robotCfg the layout `robot` section, or null (grab/dump/box then
     *     become no-ops; objects still resolve statically).
     * @return an array of scene snapshots, one per manip event.
     */
    function cargo_resolve(track, objects, robotCfg) {
        var events = (track && track.events) || [];
        objects = objects || [];
        var poseAt = (CTPS.engine && CTPS.engine.poseAt) || null;
        var state = _emptyState();
        var timeline = [];
        for (var i = 0; i < events.length; i++) {
            var ev = events[i];
            if (!ev || ev.type !== "manip") {
                continue;
            }
            var pose = poseAt ? poseAt(ev.t) : { x: 0, y: 0, h: 0 };
            _applyManip(state, ev.method, ev.arg, pose, objects, robotCfg);
            timeline.push(_snapshotState(state, i, ev.t));
        }
        return timeline;
    }

    // Exposed for store.js's loadLayout (resolve) + its before-first-event case
    // (emptyScene). store.js is inlined earlier, so it reaches these at call time.
    CTPS.cargo = CTPS.cargo || {};
    CTPS.cargo.resolve = cargo_resolve;
    CTPS.cargo.emptyScene = _emptyScene;

    // ----- canvas layers (registered on the trail canvas, below the robot) --

    /** True when object `id` is staged in a store this snapshot (omit from field). */
    function _sceneHolds(scene, id) {
        if (!scene) {
            return false;
        }
        if (scene.inHand === id || scene.rearBox === id) {
            return true;
        }
        var magazine = scene.magazine || [];
        for (var lane = 0; lane < magazine.length; lane++) {
            var stack = magazine[lane];
            for (var k = 0; k < stack.length; k++) {
                if (stack[k] === id) {
                    return true;
                }
            }
        }
        return false;
    }

    /** The object's world centre this snapshot — its `placed` spot, else authored. */
    function _sceneFieldPos(scene, obj) {
        if (scene && scene.placed &&
            Object.prototype.hasOwnProperty.call(scene.placed, obj.id)) {
            var p = scene.placed[obj.id];
            return { x: p.x_cm, y: p.y_cm };
        }
        return { x: obj.x_cm, y: obj.y_cm };
    }

    /**
     * objects layer (trail canvas): every field object at its current scene
     * position, EXCEPT those staged in a store (omitted from the field). A null
     * scene (before any layout / before the first manip event) renders every
     * object at its authored spot.
     */
    function _drawObjectsLayer(ctx, view, state) {
        if (!OBJ || !OBJ.drawObject) {
            return;
        }
        var objects = state.objects || [];
        var scene = state.scene;
        for (var i = 0; i < objects.length; i++) {
            var obj = objects[i];
            if (!obj || !obj.id) {
                continue;
            }
            if (_sceneHolds(scene, obj.id)) {
                continue;
            }
            var pos = _sceneFieldPos(scene, obj);
            OBJ.drawObject(ctx, view, obj, OBJ.project, { x: pos.x, y: pos.y });
        }
    }

    /**
     * zones layer (trail canvas): the faint robot-local manipulator overlay at
     * the interpolated pose for the current playhead. Skipped when the layout
     * carries no `robot` section.
     */
    function _drawZonesLayer(ctx, view, state) {
        if (!OBJ || !OBJ.drawZones) {
            return;
        }
        var robotCfg = state.robotCfg;
        if (!robotCfg) {
            return;
        }
        var pose = CTPS.engine.poseAt(state.t);
        OBJ.drawZones(ctx, view, pose, robotCfg, OBJ.project);
    }

    CTPS.layers.register({
        id: "objects",
        canvasId: "trail",
        visible: function (state) { return !!(state.layers && state.layers.objects); },
        draw: _drawObjectsLayer,
        invalidateOn: ["scene", "layers"],
    });
    CTPS.layers.register({
        id: "zones",
        canvasId: "trail",
        visible: function (state) { return !!(state.layers && state.layers.zones); },
        draw: _drawZonesLayer,
        invalidateOn: ["t", "scene", "layers"],
    });

    // ----- embedded-layout bootstrap ---------------------------------------
    // When a layout is embedded at emit time (the `OBJECTS` global), hand it to
    // the store once every module has registered. The in-browser upload leg (T6)
    // calls `store.loadLayout` the same way from the file input. A layout-free
    // page leaves OBJECTS undefined, so this is a no-op there.
    CTPS._inits.push(function (ctx) {
        var layout = (typeof OBJECTS !== "undefined") ? OBJECTS : null;
        var store = (ctx && ctx.store) || CTPS.store;
        if (layout && store && store.loadLayout) {
            store.loadLayout(layout);
        }
    });

    // ===================================================================
    // T6 — CARGO PANEL + UPLOAD + HUD + HOLDING LANE (Blocked By T5)
    //
    // Intentionally empty. T6 fills the two-tab cargo panel (#ctps-cargo,
    // .ctps-cargo-tab, #ctps-mag-grid, #ctps-inv-list) from store.scene /
    // store.activeCargoTab, wires #ctps-layout-file (FileReader ->
    // store.loadLayout) and #ctps-hud-holding, and rebuilds the Holding
    // timeline lane on layout-load. Do NOT implement it here.
    // ===================================================================
})();
