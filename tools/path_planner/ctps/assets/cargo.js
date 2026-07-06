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
 *   T6 (the second section) — the two-tab cargo PANEL content (the Magazine
 *     4x3 vertical-lane grid + the Inventory rear-box hold card), its live tab
 *     counts, the `#ctps-layout-file` upload wiring (FileReader ->
 *     `store.loadLayout`, tolerant per-object validation), and the
 *     `#ctps-hud-holding` HUD line. All driven by `store.scene`/
 *     `store.objects`/`store.activeCargoTab`; the tab-switching CHROME (which
 *     panel is visible, which tab button is `--active`) lives in panels.js's
 *     `renderHoldingShell` seam, the existing Log/Inspector tab idiom. The
 *     Holding timeline lane itself is timeline.js's `buildHoldingLane` seam.
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

    /**
     * Surface a resolver no-op (a manip verb that changed no cargo state) on the
     * console. Deliberately `console.warn`, NOT `CTPS._errors`: that array is a
     * one-shot boot registration channel (surfaced once, boot step 4), whereas
     * these no-ops fire per manip event on every `loadLayout` -- including
     * uploads long after boot. Messages stay short and factual.
     */
    function _noopWarn(message) {
        console.warn("cargo: " + message);
    }

    /** grab: pick the nearest on-field Tile whose centre is in the grab zone. */
    function _doGrab(state, pose, objects, robotCfg) {
        if (state.inHand !== null) {
            _noopWarn("grab ignored: manipulator already holding a tile");
            return; // in-hand occupied
        }
        var zone = robotCfg && robotCfg.grab_zone;
        if (!zone) {
            return; // no robot zones (surfaced once by the HUD note)
        }
        var tile = _nearestInZone(state, objects, TYPE_TILE, pose, zone);
        if (!tile) {
            _noopWarn("grab ignored: no tile in the grab zone");
            return; // empty-zone grab
        }
        state.inHand = tile.id;
        _clearPlaced(state, tile.id);
    }

    /** release: drop the in-hand tile back at the grab-zone world centre. */
    function _doRelease(state, pose, robotCfg) {
        if (state.inHand === null) {
            _noopWarn("release ignored: nothing in hand");
            return; // nothing to drop
        }
        var zone = robotCfg && robotCfg.grab_zone;
        if (!zone) {
            return; // no zone -> cannot place -> no-op
        }
        var w = _localToWorld(pose, zone.x_cm, zone.y_cm);
        state.placed[state.inHand] = { x_cm: w.x, y_cm: w.y };
        state.inHand = null;
    }

    /**
     * sortTo(n): append the in-hand tile to the END of magazine lane n. FIFO --
     * index 0 stays the OLDEST tile (the front / dump side) and the newest tile
     * lands last, at the top of the lane.
     */
    function _doSortTo(state, arg) {
        if (state.inHand === null) {
            _noopWarn("sortTo ignored: nothing in hand");
            return; // no in-hand
        }
        var n = _laneIndex(arg);
        if (n < 0 || n >= MAGAZINE_LANES) {
            _noopWarn("sortTo ignored: lane " + n + " out of range");
            return; // bad lane
        }
        var lane = state.magazine[n];
        if (lane.length >= MAGAZINE_DEPTH) {
            _noopWarn("sortTo ignored: lane " + n + " is full");
            return; // full lane
        }
        lane.push(state.inHand); // FIFO: newest lands last; index 0 stays oldest
        _clearPlaced(state, state.inHand);
        state.inHand = null;
    }

    /**
     * dump: eject the FRONT (oldest, index 0) tile of each non-empty lane to its
     * lane drop point. FIFO -- the tile sorted FIRST leaves FIRST.
     */
    function _doDump(state, pose, robotCfg) {
        var drops = (robotCfg && robotCfg.drop_points) || [];
        if (drops.length === 0) {
            return; // no drop points -> no-op (never lose a tile)
        }
        var ejected = 0;
        for (var lane = 0; lane < state.magazine.length; lane++) {
            var stack = state.magazine[lane];
            if (stack.length === 0) {
                continue; // empty lane -> nothing ejected
            }
            var id = stack.shift(); // FIFO: front = oldest = index 0
            var dp = drops[lane];
            var w = dp ? _localToWorld(pose, dp.x_cm, dp.y_cm)
                : { x: pose.x, y: pose.y };
            state.placed[id] = { x_cm: w.x, y_cm: w.y };
            ejected++;
        }
        if (ejected === 0) {
            _noopWarn("dump ignored: magazine is empty");
        }
    }

    /** boxClose: capture the nearest on-field Cargo whose centre is in the box zone. */
    function _doBoxClose(state, pose, objects, robotCfg) {
        if (state.rearBox !== null) {
            _noopWarn("boxClose ignored: rear box already full");
            return; // box full (REAR_BOX_CAP == 1, single slot)
        }
        var zone = robotCfg && robotCfg.box_zone;
        if (!zone) {
            return; // no zone (surfaced once by the HUD note)
        }
        var cargo = _nearestInZone(state, objects, TYPE_CARGO, pose, zone);
        if (!cargo) {
            _noopWarn("boxClose ignored: no cargo in the box zone");
            return; // no Cargo centre in zone
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
     * Each snapshot's `magazine[lane]` is ordered OLDEST-first (index 0 = oldest
     * = front = dump side): `sortTo` appends the newest tile, `dump` shifts
     * index 0, so lanes drain FIFO.
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
    // T6 — CARGO PANEL + UPLOAD + HUD (Blocked By T5)
    //
    // Fills the two-tab cargo panel (#ctps-cargo, .ctps-mag-grid,
    // #ctps-inv-list, the tab count badges) from `store.scene`/
    // `store.objects`, wires `.ctps-cargo-tab` clicks to `store.setCargoTab`,
    // wires `#ctps-layout-file` (FileReader -> tolerant validate ->
    // `store.loadLayout`), and updates `#ctps-hud-holding`. The Holding
    // timeline lane is timeline.js's seam; the tab-switch CHROME (which panel
    // is visible / which tab button is active) is panels.js's
    // `renderHoldingShell` seam, following the existing Log/Inspector idiom.
    // ===================================================================

    // ----- magazine cell label + object lookup -----------------------------

    /**
     * The magazine cell's short label: the first two non-space characters of
     * the object `name`, uppercased (Data Model: magazine cell label). Yields
     * a shorter (or empty) string for a name with fewer than 2 non-space
     * characters -- never throws.
     */
    function _shortLabel(name) {
        var text = name || "";
        var chars = [];
        for (var i = 0; i < text.length && chars.length < 2; i++) {
            var ch = text.charAt(i);
            if (ch !== " ") {
                chars.push(ch);
            }
        }
        return chars.join("").toUpperCase();
    }

    /** Find an object by id in the current layout's `objects` array, or null. */
    function _objectById(objects, id) {
        for (var i = 0; i < objects.length; i++) {
            if (objects[i] && objects[i].id === id) {
                return objects[i];
            }
        }
        return null;
    }

    // ----- counts (tabs + HUD share the same source) -----------------------

    /**
     * `{ magFilled, magCap, invFilled, invCap }` for one scene snapshot (or
     * the pristine/no-layout state when `scene` is null) -- the single source
     * both the tab badges and the HUD line read.
     */
    function _cargoCounts(scene) {
        var magFilled = 0;
        var magazine = (scene && scene.magazine) || [];
        for (var lane = 0; lane < magazine.length; lane++) {
            magFilled += magazine[lane].length;
        }
        return {
            magFilled: magFilled,
            magCap: MAGAZINE_LANES * MAGAZINE_DEPTH,
            invFilled: (scene && scene.rearBox) ? 1 : 0,
            invCap: REAR_BOX_CAP,
        };
    }

    // ----- Magazine tab: 4 vertical lanes of 3, front tile at the BOTTOM ----

    /**
     * Rebuild `#ctps-mag-grid`: one column per lane (0..MAGAZINE_LANES-1),
     * each a stack of MAGAZINE_DEPTH cells top-to-bottom. FIFO lane order:
     * `sortTo` pushes the newest tile onto the END, `dump` shifts the oldest
     * off index 0 (T5), so index 0 is the OLDEST tile (the front / dump side)
     * and it belongs at the BOTTOM row. For row `r` (0 = top .. DEPTH-1 =
     * bottom), the stack index is `MAGAZINE_DEPTH - 1 - r`: the bottom row
     * shows index 0 (oldest), each row up shows the next index, and any row
     * that maps past the stack top (index >= stack length) is an empty cell --
     * so a partially-filled lane fills from the bottom row upward, matching the
     * sidecar diagram. A filled cell gets the authored colour (via
     * `setProperty`, never a hard-coded literal) + the 2-char label; an empty
     * cell gets the dashed `--ctps-cell--empty` modifier for CSS to style.
     */
    function _renderMagazineGrid(gridEl, scene, objects) {
        if (!gridEl) {
            return;
        }
        while (gridEl.firstChild) {
            gridEl.removeChild(gridEl.firstChild);
        }
        var magazine = (scene && scene.magazine) || [];
        for (var lane = 0; lane < MAGAZINE_LANES; lane++) {
            var stack = magazine[lane] || [];
            var laneEl = document.createElement("div");
            laneEl.className = "ctps-mag-lane";

            var headerEl = document.createElement("div");
            headerEl.className = "ctps-mag-lane-header";
            headerEl.textContent = String(lane);
            laneEl.appendChild(headerEl);

            for (var row = 0; row < MAGAZINE_DEPTH; row++) {
                // FIFO: bottom row (r = DEPTH-1) shows index 0 (oldest, dump
                // side); each row up shows the next index; rows past the stack
                // top stay empty.
                var stackIndex = MAGAZINE_DEPTH - 1 - row;
                var cellEl = document.createElement("div");
                if (stackIndex >= 0 && stackIndex < stack.length) {
                    var id = stack[stackIndex];
                    var obj = _objectById(objects, id);
                    cellEl.className = "ctps-mag-cell";
                    if (obj && obj.color) {
                        cellEl.style.setProperty("--ctps-mag-cell-color", obj.color);
                    }
                    cellEl.textContent = _shortLabel(obj && obj.name);
                } else {
                    cellEl.className = "ctps-mag-cell ctps-mag-cell--empty";
                }
                laneEl.appendChild(cellEl);
            }

            var arrowEl = document.createElement("div");
            arrowEl.className = "ctps-mag-lane-arrow";
            arrowEl.textContent = "↓"; // dump-eject side, per lane
            laneEl.appendChild(arrowEl);

            gridEl.appendChild(laneEl);
        }
    }

    // ----- Inventory tab: rear-box hold card (capacity 1) -------------------

    /**
     * Rebuild `#ctps-inv-list`: the held Cargo's `.ctps-hold-card` (existing
     * CSS, `--ctps-hold` left accent) -- a colour swatch, the full object
     * name, and an `onrobot` pill -- or, when the rear box is empty/closed, a
     * one-line hint (verbatim from the UI sidecar).
     */
    function _renderInventoryPanel(listEl, scene, objects) {
        if (!listEl) {
            return;
        }
        while (listEl.firstChild) {
            listEl.removeChild(listEl.firstChild);
        }
        var id = scene && scene.rearBox;
        if (!id) {
            var hint = document.createElement("div");
            hint.className = "ctps-inv-empty";
            hint.textContent = "closed — captures Cargo in the rear-box zone";
            listEl.appendChild(hint);
            return;
        }
        var obj = _objectById(objects, id);
        var card = document.createElement("div");
        card.className = "ctps-hold-card";

        var swatch = document.createElement("span");
        swatch.className = "ctps-hold-card-swatch";
        if (obj && obj.color) {
            swatch.style.setProperty("--ctps-swatch-color", obj.color);
        }
        card.appendChild(swatch);

        var name = document.createElement("span");
        name.className = "ctps-hold-card-name";
        name.textContent = (obj && obj.name) ? obj.name : id;
        card.appendChild(name);

        var pill = document.createElement("span");
        pill.className = "ctps-hold-card-pill";
        pill.textContent = "onrobot";
        card.appendChild(pill);

        listEl.appendChild(card);
    }

    // ----- tab counts + HUD --------------------------------------------------

    /** "N/CAP" on each tab badge, matching the sidecar mockup's count form. */
    function _renderCargoTabCounts(dom, scene) {
        var counts = _cargoCounts(scene);
        if (dom.countMag) {
            dom.countMag.textContent = counts.magFilled + "/" + counts.magCap;
        }
        if (dom.countInv) {
            dom.countInv.textContent = counts.invFilled + "/" + counts.invCap;
        }
    }

    /** "magazine N/CAP  rear box N/CAP" on the stage HUD -- no tab spacing. */
    function _renderCargoHud(dom, scene) {
        if (!dom.hudHolding) {
            return;
        }
        var counts = _cargoCounts(scene);
        dom.hudHolding.textContent =
            "magazine " + counts.magFilled + "/" + counts.magCap +
            "  rear box " + counts.invFilled + "/" + counts.invCap;
    }

    // ----- "no robot zones" HUD note -----------------------------------------

    /** True when the layout's robot section defines at least one manipulator zone. */
    function _hasRobotZones(robotCfg) {
        return !!(robotCfg && (robotCfg.grab_zone || robotCfg.box_zone));
    }

    /**
     * Lazily create (once) the one-line note that sits right after
     * `#ctps-hud-holding`, flagging a loaded layout whose `robot` section
     * carries no manipulator zones (grab/box/dump then all no-op). Returns null
     * when the HUD line is absent (a layout-free page) so callers no-op safely.
     */
    function _ensureRobotNoteEl() {
        var el = document.getElementById("ctps-hud-robotnote");
        if (el) {
            return el;
        }
        var anchor = document.getElementById("ctps-hud-holding");
        if (!anchor || !anchor.parentNode) {
            return null;
        }
        el = document.createElement("span");
        el.id = "ctps-hud-robotnote";
        el.className = "ctps-hud-robotnote";
        el.hidden = true;
        anchor.parentNode.insertBefore(el, anchor.nextSibling);
        return el;
    }

    /**
     * Show "no robot zones" beside the HUD when a layout is loaded but its robot
     * config defines no zones; hide it otherwise. Gated on a non-empty `objects`
     * array so the empty startup player (no layout at all) stays silent. Sets
     * textContent + the `hidden` flag only -- never an inline style.
     */
    function _renderRobotZonesNote(state) {
        var el = _ensureRobotNoteEl();
        if (!el) {
            return;
        }
        var objects = state.objects || [];
        if (objects.length > 0 && !_hasRobotZones(state.robotCfg)) {
            el.textContent = "no robot zones";
            el.hidden = false;
        } else {
            el.textContent = "";
            el.hidden = true;
        }
    }

    // ----- content orchestration --------------------------------------------

    /** Rebuild every scene-driven piece of the cargo panel + HUD in one pass. */
    function _renderCargoContent(dom, state) {
        var scene = state.scene;
        var objects = state.objects || [];
        _renderMagazineGrid(dom.magGrid, scene, objects);
        _renderInventoryPanel(dom.invList, scene, objects);
        _renderCargoTabCounts(dom, scene);
        _renderCargoHud(dom, scene);
        _renderRobotZonesNote(state);
    }

    function _queryCargoDom() {
        return {
            magGrid: document.getElementById("ctps-mag-grid"),
            invList: document.getElementById("ctps-inv-list"),
            countMag: document.getElementById("ctps-cargo-count-mag"),
            countInv: document.getElementById("ctps-cargo-count-inv"),
            hudHolding: document.getElementById("ctps-hud-holding"),
        };
    }

    // ----- tab clicks -> store.setCargoTab -----------------------------------

    /**
     * Wire every `.ctps-cargo-tab[data-cargo-tab]` button to
     * `store.setCargoTab`. The resulting `notify(["activeCargoTab"])` is
     * consumed by panels.js's `renderHoldingShell` (which panel/tab-button
     * becomes active) -- this module only dispatches the action.
     */
    function _wireCargoTabs(store) {
        var tabs = document.querySelectorAll(".ctps-cargo-tab[data-cargo-tab]");
        for (var i = 0; i < tabs.length; i++) {
            tabs[i].addEventListener("click", function (evt) {
                store.setCargoTab(evt.currentTarget.getAttribute("data-cargo-tab"));
            });
        }
    }

    // ----- layout upload: FileReader -> tolerant validate -> store.loadLayout

    /**
     * Minimal tolerant shape check for one uploaded layout object: enough to
     * keep `cargo_resolve`/`CTPS_OBJ.drawObject` from choking on a malformed
     * entry. Mirrors the loader convention (skip + warn), NOT `layout.py`'s
     * loud strict validation -- that lives at emit time, not here.
     */
    var _VALID_SHAPES = { rect: true, circle: true };
    var _VALID_TYPES = { tile: true, cargo: true, fixed: true };

    function _isValidLayoutObject(obj) {
        if (!obj || typeof obj !== "object") {
            return false;
        }
        if (typeof obj.id !== "string" || obj.id === "") {
            return false;
        }
        if (typeof obj.shape !== "string" || !_VALID_SHAPES[obj.shape]) {
            return false;
        }
        if (typeof obj.type !== "string" || !_VALID_TYPES[obj.type]) {
            return false;
        }
        if (typeof obj.x_cm !== "number" || typeof obj.y_cm !== "number") {
            return false;
        }
        return true;
    }

    /**
     * Lazily create (once) the inline "couldn't read layout" note, inserted
     * right after the upload button in the header. Not part of the T4 DOM
     * contract (no reserved slot exists for it) -- created on first need
     * rather than left as static markup that's usually hidden.
     */
    function _ensureLayoutErrorEl() {
        var el = document.getElementById("ctps-layout-error");
        if (el) {
            return el;
        }
        el = document.createElement("span");
        el.id = "ctps-layout-error";
        el.className = "ctps-layout-error";
        el.hidden = true;
        var anchor = document.getElementById("ctps-layout-upload-btn");
        if (anchor && anchor.parentNode) {
            anchor.parentNode.insertBefore(el, anchor.nextSibling);
        } else {
            document.body.appendChild(el);
        }
        return el;
    }

    function _showLayoutError(message) {
        var el = _ensureLayoutErrorEl();
        el.textContent = message;
        el.hidden = false;
    }

    function _clearLayoutError() {
        var el = document.getElementById("ctps-layout-error");
        if (el) {
            el.hidden = true;
            el.textContent = "";
        }
    }

    /**
     * Wire `#ctps-layout-file`'s `change` -> FileReader -> `store.loadLayout`
     * (the in-browser upload leg of AC5). Error handling is 2-tier: a
     * structurally invalid file (bad JSON, or no `objects` array) shows the
     * inline note and leaves `store` untouched; a malformed INDIVIDUAL object
     * inside an otherwise-valid file is skipped (`console.warn`) and every
     * other object still loads. Never uses localStorage / the File System
     * Access API (dead on `file://`).
     */
    function _wireLayoutUpload(store) {
        var input = document.getElementById("ctps-layout-file");
        if (!input) {
            return;
        }
        // The visible "Load layout…" button opens the dialog; the file input
        // carries the `hidden` attribute, so nothing opens unless the button
        // click forwards to `input.click()`. Both are guarded so a layout-free
        // page (neither element present) never throws.
        var uploadBtn = document.getElementById("ctps-layout-upload-btn");
        if (uploadBtn) {
            uploadBtn.addEventListener("click", function () {
                input.click();
            });
        }
        input.addEventListener("change", function () {
            var file = input.files && input.files[0];
            input.value = ""; // allow re-choosing the same file to re-fire 'change'
            if (!file) {
                return;
            }
            var reader = new FileReader();
            reader.onload = function () {
                var parsed;
                try {
                    parsed = JSON.parse(String(reader.result));
                } catch (parseError) {
                    _showLayoutError("Couldn't read layout: invalid JSON.");
                    return;
                }
                if (!parsed || typeof parsed !== "object" ||
                    !Array.isArray(parsed.objects)) {
                    _showLayoutError("Couldn't read layout: missing \"objects\" array.");
                    return;
                }
                var validObjects = [];
                for (var i = 0; i < parsed.objects.length; i++) {
                    var obj = parsed.objects[i];
                    if (_isValidLayoutObject(obj)) {
                        validObjects.push(obj);
                    } else {
                        console.warn(
                            "cargo: skipping malformed layout object at index " + i, obj);
                    }
                }
                _clearLayoutError();
                store.loadLayout({ objects: validObjects, robot: parsed.robot });
            };
            reader.onerror = function () {
                _showLayoutError("Couldn't read layout: file read error.");
            };
            reader.readAsText(file);
        });
    }

    // ----- boot ---------------------------------------------------------------

    /**
     * Wire the cargo panel's tab clicks + upload input, paint the panel once
     * from whatever `store` already holds (the T5 embedded-layout bootstrap,
     * pushed earlier in this same file, has already run by the time this
     * fires -- so an embedded layout is already reflected), then subscribe so
     * every later `scene`/`objects` change (an upload, or the t->scene
     * bisect) repaints it.
     */
    function _initCargoPanel(ctx) {
        var store = (ctx && ctx.store) || CTPS.store;
        if (!store) {
            return;
        }
        var dom = _queryCargoDom();
        _wireCargoTabs(store);
        _wireLayoutUpload(store);
        _renderCargoContent(dom, store);
        store.subscribe(function (changedKeys, state) {
            if (changedKeys.indexOf("scene") !== -1 ||
                changedKeys.indexOf("objects") !== -1) {
                _renderCargoContent(dom, state);
            }
        });
    }

    CTPS._inits.push(_initCargoPanel);
})();
