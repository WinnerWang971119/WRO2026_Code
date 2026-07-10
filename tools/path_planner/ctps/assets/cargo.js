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
 *     cargo timeline lanes (MAGAZINE / REAR BOX) are timeline.js's
 *     `buildMagazineLane` / `buildRearBoxLane` seams.
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

    // Mirror of ctps/config.py::MANIP_NOMINAL_S — the on-screen dwell of a manip
    // event; a deploy flash spans [ev.t, ev.t + MANIP_NOMINAL_S].
    var MANIP_NOMINAL_S = 0.5;

    // Trace manip method values — exact strings (Contracts: Naming; recorder
    // <-> resolver). `reset` intentionally changes no cargo state.
    var METHOD_GRAB = "grab";
    var METHOD_RELEASE = "release";  // recorded, but a no-op here (see `_applyManip`)
    var METHOD_SORT_TO = "sortTo";
    var METHOD_DUMP = "dump";
    var METHOD_BOX_CLOSE = "boxClose";
    var METHOD_BOX_OPEN = "boxOpen";

    var TYPE_TILE = "tile";
    var TYPE_CARGO = "cargo";

    // ----- working-state helpers -------------------------------------------

    /**
     * A fresh mutable working state: sorter unaimed (`sorterLane: null`, so the
     * first grab MUST be preceded by a `sortTo`), all lanes empty, box empty.
     */
    function _emptyState() {
        var magazine = [];
        for (var lane = 0; lane < MAGAZINE_LANES; lane++) {
            magazine.push([]);
        }
        return { sorterLane: null, magazine: magazine, rearBox: null, placed: {} };
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
            sorterLane: state.sorterLane,
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

    // Robot-local -> world reuses objects_draw.js's `CTPS_OBJ.localToWorld`
    // (loaded before this file): ONE copy of the rotation math on the page, so a
    // sign/handedness fix can never desync the cargo/zone placement here from the
    // object drawing there.
    var _localToWorld = OBJ.localToWorld;

    /** True when a robot-local point falls inside the axis-aligned local zone rect. */
    function _pointInZone(local, zone) {
        return Math.abs(local.x - zone.x_cm) <= zone.w_cm / 2 &&
            Math.abs(local.y - zone.y_cm) <= zone.h_cm / 2;
    }

    // ----- object-position helpers -----------------------------------------

    /** True when object `id` is NOT staged in any store (magazine / box). */
    function _isOnField(state, id) {
        if (state.rearBox === id) {
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
     * Squared distance from a robot-local point to the robot body footprint.
     *
     * The footprint is treated as an axis-aligned square in robot-local space
     * with half-side `robotHalfSideCm` (the current chassis heading is already
     * factored out by `_worldToLocal`). When the point is inside/on the body,
     * distance is 0.
     */
    function _distSqToRobotBody(local, robotHalfSideCm) {
        var half = (typeof robotHalfSideCm === "number" && robotHalfSideCm > 0)
            ? robotHalfSideCm : 0;
        var dx = Math.max(Math.abs(local.x) - half, 0);
        var dy = Math.max(Math.abs(local.y) - half, 0);
        return dx * dx + dy * dy;
    }

    /**
     * The robot footprint half-side in cm. Prefers `field.robot_frame_cm`
     * (physical chassis square), falls back to `field.robot_radius_cm` (legacy
     * traces), then 0 when missing.
     */
    function _robotHalfSideCm() {
        var track = (CTPS.engine && CTPS.engine.track) || null;
        var field = (track && track.field) || null;
        var frame = field && field.robot_frame_cm;
        if (typeof frame === "number" && frame > 0) {
            return frame / 2;
        }
        var radius = field && field.robot_radius_cm;
        if (typeof radius === "number" && radius > 0) {
            return radius;
        }
        return 0;
    }

    /**
     * The nearest on-field object of `type` whose CURRENT centre falls inside
     * `zone` at `pose` — or null when none qualifies. "Nearest" is measured to
     * the provided local-frame reference point (`refX`, `refY`) by default; when
     * `robotHalfSideCm` is provided, nearest is measured to the robot body
     * footprint (distance to the chassis rectangle, not distance to origin).
     */
    function _nearestInZone(state, objects, type, pose, zone, refX, refY, robotHalfSideCm) {
        var anchorX = (typeof refX === "number") ? refX : zone.x_cm;
        var anchorY = (typeof refY === "number") ? refY : zone.y_cm;
        var useRobotBody = (typeof robotHalfSideCm === "number");
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
            var dist;
            if (useRobotBody) {
                dist = _distSqToRobotBody(local, robotHalfSideCm);
            } else {
                var ddx = local.x - anchorX;
                var ddy = local.y - anchorY;
                dist = ddx * ddx + ddy * ddy;
            }
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
    // When true, `_noopWarn` and the grab-before-sortTo `console.error` stay
    // silent. Set for the duration of a SILENT `cargo_resolve` (the timeline
    // lane builders re-resolve on every scrub, and would otherwise re-emit the
    // resolver's warnings/errors per builder, per boundary crossing).
    var _suppressResolveLog = false;

    function _noopWarn(message) {
        if (_suppressResolveLog) {
            return;
        }
        console.warn("cargo: " + message);
    }

    /**
     * grab: capture the nearest on-field Tile whose centre is in the grab zone
     * and deposit it straight into the sorter-selected magazine lane
     * (`state.sorterLane`). Mirrors the real mechanism: `sortTo` pre-aims the
     * sorter servo and the FOLLOWING grab feeds the tile into that lane
     * (Manipulator.h -- sortTo swings the sorter, grab closes the grabber).
     *
     * A grab with NO lane selected is an OPERATOR ERROR, not a benign no-op: the
     * tile is left on the field (unsorted, so the mistake is visible) and an
     * error string is returned for the caller to surface loudly. A full selected
     * lane or an empty grab zone stays a quiet no-op warning -- never an error,
     * never a lost tile.
     *
     * @return an error message when grab fired with no sorter lane selected;
     *         otherwise null.
     */
    function _doGrab(state, pose, objects, robotCfg) {
        var zone = robotCfg && robotCfg.grab_zone;
        if (!zone) {
            return null; // no robot zones (surfaced once by the HUD note)
        }
        // "Nearest" for grab is robot-body proximity, while still requiring
        // the tile centre to lie inside the grab zone.
        var tile = _nearestInZone(
            state,
            objects,
            TYPE_TILE,
            pose,
            zone,
            0,
            0,
            _robotHalfSideCm()
        );
        if (!tile) {
            _noopWarn("grab ignored: no tile in the grab zone");
            return null; // empty-zone grab
        }
        if (state.sorterLane === null) {
            // Sticky-sorter contract: you MUST sortTo (aim the sorter) before a
            // grab. Leave the tile on the field so the un-sorted tile is visible
            // and hand the caller a loud, persistent operator error.
            return "grab before sortTo: no magazine lane selected";
        }
        var lane = state.magazine[state.sorterLane];
        if (lane.length >= MAGAZINE_DEPTH) {
            _noopWarn("grab ignored: sorter lane " + state.sorterLane + " is full");
            return null; // full lane -> tile stays on the field (never lost)
        }
        lane.push(tile.id); // FIFO: newest lands last; dump shifts index 0
        _clearPlaced(state, tile.id);
        return null;
    }

    /**
     * sortTo(n): AIM the sorter at magazine lane n (sticky selection). Mirrors
     * the real mechanism, where `sortTo` swings the sorter servo to pre-select
     * the destination lane and the FOLLOWING `grab` feeds the tile into it. Moves
     * no tile itself -- it only records `state.sorterLane`, which persists until
     * the next `sortTo`, so several grabs can feed one lane. An out-of-range lane
     * is a no-op warning and leaves the current selection unchanged.
     */
    function _doSortTo(state, arg) {
        var n = _laneIndex(arg);
        if (n < 0 || n >= MAGAZINE_LANES) {
            _noopWarn("sortTo ignored: lane " + n + " out of range");
            return; // bad lane -> selection unchanged
        }
        state.sorterLane = n; // sticky: holds until the next sortTo
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
     * Apply one manip verb to the working state at `pose`. `release`, `reset`,
     * and unknown methods leave the state unchanged.
     *
     * @return an operator-error message when the verb was an invalid grab (grab
     *         with no sorter lane selected); otherwise null.
     */
    function _applyManip(state, method, arg, pose, objects, robotCfg) {
        if (method === METHOD_GRAB) {
            return _doGrab(state, pose, objects, robotCfg);
        } else if (method === METHOD_RELEASE) {
            // No-op by design: in the fused grab+sort magazine model a grab feeds
            // the tile straight into a lane, so there is never a loose in-hand
            // tile for `release` to drop back -- `dump` is how tiles leave the
            // magazine. Kept as an explicit branch so a recorded `release` reads
            // as intentionally ignored, not as a forgotten/unknown method.
            return null;
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
        return null;
    }

    /**
     * Resolve the whole scene timeline in one pure pass: one snapshot per `manip`
     * event, in event order. Each event is applied against the robot pose at its
     * own time (`CTPS.engine.poseAt`), which projects the grab/box zones and drop
     * points to world. No-op verbs still emit a snapshot (mirroring the prior
     * state) so every manip event has a scene index for the store's bisect.
     * Each snapshot's `magazine[lane]` is ordered OLDEST-first (index 0 = oldest
     * = front = dump side): `sortTo` aims the sorter and `grab` appends the newest
     * tile to the selected lane, `dump` shifts index 0, so lanes drain FIFO.
     *
     * A grab fired with no sorter lane selected (a missing `sortTo`) is an
     * operator error: it is `console.error`-logged with its event index, and the
     * FIRST such message is hoisted onto the returned array as `.sorterError`
     * (store.js reads it into `store.sorterError` to drive the persistent HUD
     * note).
     *
     * @param track the ingested Data-Model-B track (reads `track.events`).
     * @param objects the layout objects array (`store.objects`).
     * @param robotCfg the layout `robot` section, or null (grab/dump/box then
     *     become no-ops; objects still resolve statically).
     * @param silent when truthy, suppress the resolver's per-event `console.warn`
     *     no-ops and the grab-before-sortTo `console.error`. `store.loadLayout`
     *     resolves once NON-silently (so the operator sees each issue exactly
     *     once); the timeline lane builders, which re-resolve on every scrub,
     *     pass `silent: true` so a single underlying event is not logged again.
     *     The `.sorterError` hoist is unaffected -- the HUD note still fires.
     * @return an array of scene snapshots, one per manip event, carrying a
     *     `.sorterError` string (or null) for the grab-before-sortTo note.
     */
    function cargo_resolve(track, objects, robotCfg, silent) {
        var events = (track && track.events) || [];
        objects = objects || [];
        var poseAt = (CTPS.engine && CTPS.engine.poseAt) || null;
        var state = _emptyState();
        var timeline = [];
        var sorterError = null;
        var prevSuppress = _suppressResolveLog;
        _suppressResolveLog = !!silent;
        try {
            for (var i = 0; i < events.length; i++) {
                var ev = events[i];
                if (!ev || ev.type !== "manip") {
                    continue;
                }
                var pose = poseAt ? poseAt(ev.t) : { x: 0, y: 0, h: 0 };
                var err = _applyManip(state, ev.method, ev.arg, pose, objects, robotCfg);
                if (err) {
                    if (!silent) {
                        console.error("cargo: " + err + " (event " + i + ")");
                    }
                    if (sorterError === null) {
                        sorterError = err; // first offending grab drives the HUD note
                    }
                }
                timeline.push(_snapshotState(state, i, ev.t));
            }
        } finally {
            _suppressResolveLog = prevSuppress;
        }
        timeline.sorterError = sorterError;
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
        if (scene.rearBox === id) {
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

    // ----- deploy layer: live manipulator-area overlay ----------------------
    // Per-actuator "deployed" semantics: the grab area flashes on RELEASE events
    // (not `grab`), the dump area flashes on `dump` events, and the box area is
    // shown across each rear-box hold — the `boxClose` EVENT shows it and the
    // matching `boxOpen` EVENT clears it. All three are derived purely from the
    // fixed track events (never the cargo-capture model), so they never change
    // after ingest and are computed ONCE (cached) rather than rescanned every
    // repaint — mirroring chrome.js's compute-once `findWorstDrift` idiom.
    //
    // NOTE: the box overlay is EVENT-driven, not cargo-driven. It shows whenever
    // the rear box is closed, independent of whether a Cargo object happened to
    // lie in the box zone at `boxClose` time (which the cargo model requires to
    // set `scene.rearBox`). Keying it to the events makes the overlay reflect
    // the actuator's deployment on any trace, regardless of layout alignment.

    // { release: [[t0,t1],...], dump: [[t0,t1],...], box: [[close,open),...] },
    // lazily built + cached on first `_deployMaskAt` call (CTPS.engine.events is
    // not necessarily ready at IIFE-eval time, so this cannot be precomputed at
    // module load).
    var _deployWindows = null;

    /**
     * Build the release/dump flash-window lists and the box close->open spans
     * from `CTPS.engine.events` in a single pass. Each `manip` event is
     * `method`-gated (never the display `label`), so a legacy trace whose events
     * carry only `label` yields empty lists and the deploy layer draws nothing.
     * Every grab/dump window spans the manip event's on-screen dwell,
     * `[ev.t, ev.t + MANIP_NOMINAL_S]`; every box span runs `[boxClose.t,
     * boxOpen.t)` (close shows, open clears). A `boxClose` with no later
     * `boxOpen` stays deployed to the trace end (`Infinity`); a redundant
     * `boxClose` while already closed, or a `boxOpen` with nothing closed, is a
     * no-op — mirroring the single-slot rear box.
     */
    function _buildDeployWindows() {
        var events = (CTPS.engine && CTPS.engine.events) || [];
        var release = [];
        var dump = [];
        var box = [];
        var pendingClose = null; // time of an unmatched boxClose, or null
        for (var i = 0; i < events.length; i++) {
            var ev = events[i];
            if (!ev || ev.type !== "manip") {
                continue;
            }
            if (ev.method === METHOD_RELEASE) {
                release.push([ev.t, ev.t + MANIP_NOMINAL_S]);
            } else if (ev.method === METHOD_DUMP) {
                dump.push([ev.t, ev.t + MANIP_NOMINAL_S]);
            } else if (ev.method === METHOD_BOX_CLOSE) {
                if (pendingClose === null) {
                    pendingClose = ev.t; // first close of a hold; later closes no-op
                }
            } else if (ev.method === METHOD_BOX_OPEN) {
                if (pendingClose !== null) {
                    box.push([pendingClose, ev.t]);
                    pendingClose = null;
                }
            }
        }
        if (pendingClose !== null) {
            box.push([pendingClose, Infinity]); // closed, never opened -> to end
        }
        return { release: release, dump: dump, box: box };
    }

    /** The cached deploy-window lists, building them once on first use. */
    function _deployWindowsCached() {
        if (!_deployWindows) {
            _deployWindows = _buildDeployWindows();
        }
        return _deployWindows;
    }

    /** True when `t` falls inside any `[t0, t1]` window (inclusive both ends). */
    function _tInAnyWindow(t, windows) {
        for (var i = 0; i < windows.length; i++) {
            var w = windows[i];
            if (t >= w[0] && t <= w[1]) {
                return true;
            }
        }
        return false;
    }

    /**
     * True when `t` falls inside any `[close, open)` box span — start-inclusive
     * so the `boxClose` frame already shows the box, end-exclusive so the
     * `boxOpen` frame clears it (an open with no later close runs to `Infinity`).
     */
    function _tInAnyBoxSpan(t, spans) {
        for (var i = 0; i < spans.length; i++) {
            var s = spans[i];
            if (t >= s[0] && t < s[1]) {
                return true;
            }
        }
        return false;
    }

    /**
     * Resolve which manipulator areas are deployed at time `t`: `grab` flashes
     * on a cached `release` window, `dump` flashes on a cached `dump` window,
     * and `box` shows across a cached `boxClose`->`boxOpen` span (close shows,
     * open clears). All three read only the cached, event-derived windows — the
     * box is intentionally NOT gated on the cargo model's `scene.rearBox`, so it
     * reflects the actuator on any trace regardless of layout alignment.
     *
     * @param t the current playhead time (s).
     * @return `{ grab, box, dump }` booleans.
     */
    function _deployMaskAt(t) {
        var windows = _deployWindowsCached();
        return {
            grab: _tInAnyWindow(t, windows.release),
            box: _tInAnyBoxSpan(t, windows.box),
            dump: _tInAnyWindow(t, windows.dump),
        };
    }

    /**
     * deploy layer (trail canvas): the filled-accent live manipulator overlay,
     * drawn only for the areas deployed at the current playhead — see
     * `_deployMaskAt`. No-ops with no loaded layout/robot section, or when
     * every mask flag is false (AC5). Registered AFTER `zones` so its accent
     * fill paints above the static dashed overlay.
     */
    function _drawDeployLayer(ctx, view, state) {
        if (!OBJ || !OBJ.drawDeployedZones) {
            return;
        }
        var robotCfg = state.robotCfg;
        if (!robotCfg) {
            return;
        }
        var mask = _deployMaskAt(state.t);
        if (!mask.grab && !mask.box && !mask.dump) {
            return;
        }
        var pose = CTPS.engine.poseAt(state.t);
        OBJ.drawDeployedZones(ctx, view, pose, robotCfg, OBJ.project, mask);
    }

    CTPS.layers.register({
        id: "deploy",
        canvasId: "trail",
        visible: function (state) { return !!(state.layers && state.layers.deploy); },
        draw: _drawDeployLayer,
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
     * characters -- never throws. Iterates by CODE POINT (walking surrogate
     * pairs as one unit) so an emoji/astral name (e.g. a red-square emoji then
     * "R") renders a whole glyph, not a lone-surrogate replacement box.
     */
    function _shortLabel(name) {
        var text = name || "";
        var chars = [];
        for (var i = 0; i < text.length && chars.length < 2; ) {
            var code = text.charCodeAt(i);
            var size = (code >= 0xD800 && code <= 0xDBFF && i + 1 < text.length) ? 2 : 1;
            var ch = text.substr(i, size);
            i += size;
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
     * `grab` pushes the newest tile onto the END of the sorter-selected lane,
     * `dump` shifts the oldest off index 0 (T5), so index 0 is the OLDEST tile
     * (the front / dump side) and it belongs at the BOTTOM row. For row `r`
     * (0 = top .. DEPTH-1 = bottom), the stack index is `MAGAZINE_DEPTH - 1 - r`:
     * the bottom row shows index 0 (oldest), each row up shows the next index,
     * and any row that maps past the stack top (index >= stack length) is an
     * empty cell -- so a partially-filled lane fills from the bottom row upward,
     * matching the sidecar diagram. A filled cell gets the authored colour (via
     * `setProperty`, never a hard-coded literal) + the 2-char label; an empty
     * cell gets the dashed `--ctps-cell--empty` modifier for CSS to style.
     *
     * The lane the sorter is currently aimed at (`scene.sorterLane`, set by the
     * sticky `sortTo`) gets a ▼ caret above it and an accent-coloured header
     * (`--selected` modifier). Every other lane emits an EMPTY caret row so all
     * headers stay vertically aligned; before the first `sortTo`
     * (`sorterLane` null) no lane is marked.
     */
    function _renderMagazineGrid(gridEl, scene, objects) {
        if (!gridEl) {
            return;
        }
        while (gridEl.firstChild) {
            gridEl.removeChild(gridEl.firstChild);
        }
        var magazine = (scene && scene.magazine) || [];
        var selectedLane = (scene && typeof scene.sorterLane === "number")
            ? scene.sorterLane : -1;
        for (var lane = 0; lane < MAGAZINE_LANES; lane++) {
            var stack = magazine[lane] || [];
            var laneEl = document.createElement("div");
            laneEl.className = "ctps-mag-lane";

            // Sorter caret: ▼ over the aimed lane only; an empty row otherwise so
            // every lane's header sits at the same height.
            var caretEl = document.createElement("div");
            caretEl.className = "ctps-mag-lane-caret";
            caretEl.textContent = (lane === selectedLane) ? "▼" : "";
            laneEl.appendChild(caretEl);

            var headerEl = document.createElement("div");
            headerEl.className = "ctps-mag-lane-header";
            if (lane === selectedLane) {
                headerEl.className += " ctps-mag-lane-header--selected";
            }
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

    // ----- grab-before-sortTo error note -------------------------------------

    /**
     * Lazily create (once) the persistent sorter-error banner, appended inside
     * the always-visible cargo panel (`#ctps-cargo`) below its two tab bodies --
     * NOT the stage HUD, which this player hides by default. Surfaced whenever
     * the loaded trace fired a `grab` with no sorter lane selected (a missing
     * `sortTo`) -- `--ctps-drift` reads as an error. Appending to `#ctps-cargo`
     * (not the magazine-only body) keeps it visible on either cargo tab. Returns
     * null when the cargo panel is absent (a layout-free page) so callers no-op.
     */
    function _ensureSorterErrorEl() {
        var el = document.getElementById("ctps-cargo-error");
        if (el) {
            return el;
        }
        var panel = document.getElementById("ctps-cargo");
        if (!panel) {
            return null;
        }
        el = document.createElement("div");
        el.id = "ctps-cargo-error";
        el.className = "ctps-cargo-error";
        el.hidden = true;
        panel.appendChild(el);
        return el;
    }

    /**
     * Show the sorter-error note when the resolver flagged a grab-before-sortTo
     * in the loaded trace (`store.sorterError`), hide it otherwise. This note is
     * PERSISTENT -- it stays visible at every playhead position while the loaded
     * trace carries the error, so the operator cannot scrub past it unseen. Sets
     * textContent + the `hidden` flag only -- never an inline style.
     */
    function _renderSorterErrorNote(state) {
        var el = _ensureSorterErrorEl();
        if (!el) {
            return;
        }
        if (state.sorterError) {
            el.textContent = "cargo error: " + state.sorterError;
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
        _renderSorterErrorNote(state);
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

    // ----- tolerant per-object + robot coercion -----------------------------
    // The upload leg mirrors the EDITOR's tolerant loader (editor_io.js
    // `_parseObject` / `_parseRobot`), NOT `layout.py`'s loud strict validation
    // (that lives at emit time). Both tolerant paths must AGREE so the same
    // hand-edited layout.json loads identically in the editor and the player: a
    // missing `type` defaults to `tile`, missing shape dimensions default to a
    // visible size (never a 0x0 object that silently vanishes), and a non-finite
    // robot zone dimension is DROPPED rather than fed to `_pointInZone` as NaN
    // (which would make every `<=` test false and silently break all grabbing).
    var _VALID_TYPES = { tile: true, cargo: true, fixed: true };
    var _HEX_RE = /^#[0-9a-fA-F]{6}$/;

    /** True for a finite JS number (not NaN/Infinity, not a bool). */
    function _finiteNum(v) {
        return typeof v === "number" && isFinite(v);
    }

    /**
     * Coerce one uploaded layout object into the normalized shape the resolver
     * and `drawObject` expect, or null when it is unusable (not an object, an
     * unknown shape, or a non-finite centre). Defaults a missing `type` to
     * `tile`, a missing colour to grey, a missing id to `oN`, and missing shape
     * dimensions to a visible fallback -- matching the editor's `_parseObject`
     * policy so the two tolerant loaders never disagree on the same file.
     */
    function _coerceLayoutObject(o, i, seen) {
        if (!o || typeof o !== "object") {
            return null;
        }
        if (o.shape !== "rect" && o.shape !== "circle") {
            return null;
        }
        if (!_finiteNum(o.x_cm) || !_finiteNum(o.y_cm)) {
            return null;
        }
        var id = (typeof o.id === "string" && o.id) ? o.id : ("o" + (i + 1));
        if (seen[id]) {
            id = id + "_" + i;
        }
        seen[id] = true;
        var type = _VALID_TYPES[o.type] ? o.type : "tile";
        var color = (typeof o.color === "string" && _HEX_RE.test(o.color)) ? o.color : "#888888";
        var obj = {
            id: id,
            name: (typeof o.name === "string") ? o.name : id,
            shape: o.shape,
            type: type,
            color: color,
            x_cm: +o.x_cm,
            y_cm: +o.y_cm,
            heading_deg: _finiteNum(o.heading_deg) ? +o.heading_deg : 0,
        };
        if (o.shape === "rect") {
            obj.w_cm = (_finiteNum(o.w_cm) && o.w_cm > 0) ? +o.w_cm : 5;
            obj.h_cm = (_finiteNum(o.h_cm) && o.h_cm > 0) ? +o.h_cm : 5;
        } else {
            obj.radius_cm = (_finiteNum(o.radius_cm) && o.radius_cm > 0) ? +o.radius_cm : 3;
        }
        return obj;
    }

    /** Parse one robot-local rect zone (finite centre + positive extent), or null. */
    function _coerceZone(z) {
        if (!z || typeof z !== "object") {
            return null;
        }
        if (!_finiteNum(z.x_cm) || !_finiteNum(z.y_cm) ||
            !_finiteNum(z.w_cm) || !_finiteNum(z.h_cm) ||
            z.w_cm <= 0 || z.h_cm <= 0) {
            return null;
        }
        return { x_cm: +z.x_cm, y_cm: +z.y_cm, w_cm: +z.w_cm, h_cm: +z.h_cm };
    }

    /**
     * Numerically sanitize an uploaded `robot` section: keep only zones whose
     * dimensions are finite and positive, and drop-point entries with finite
     * coordinates. A zone that fails validation is OMITTED (its verb then
     * no-ops, flagged by the "no robot zones" note) rather than passed through
     * with a NaN dimension that would make `_pointInZone` reject every candidate
     * and silently break all grabbing. Returns null when the section is absent
     * or carries nothing usable.
     */
    function _coerceRobotCfg(robot) {
        if (!robot || typeof robot !== "object") {
            return null;
        }
        var grab = _coerceZone(robot.grab_zone);
        var box = _coerceZone(robot.box_zone);
        var drops = [];
        if (Array.isArray(robot.drop_points)) {
            for (var i = 0; i < robot.drop_points.length; i++) {
                var d = robot.drop_points[i];
                if (d && typeof d === "object" && _finiteNum(d.x_cm) && _finiteNum(d.y_cm)) {
                    drops.push({ x_cm: +d.x_cm, y_cm: +d.y_cm });
                }
            }
        }
        if (!grab && !box && drops.length === 0) {
            return null;
        }
        var out = {};
        if (grab) {
            out.grab_zone = grab;
        }
        if (box) {
            out.box_zone = box;
        }
        if (drops.length > 0) {
            out.drop_points = drops;
        }
        return out;
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
                var seen = {};
                for (var i = 0; i < parsed.objects.length; i++) {
                    var obj = _coerceLayoutObject(parsed.objects[i], i, seen);
                    if (obj) {
                        validObjects.push(obj);
                    } else {
                        console.warn(
                            "cargo: skipping malformed layout object at index " + i,
                            parsed.objects[i]);
                    }
                }
                _clearLayoutError();
                store.loadLayout({
                    objects: validObjects,
                    robot: _coerceRobotCfg(parsed.robot),
                });
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
