/* CTPS — Clear Trajectory Prediction Simulator: player store.
 *
 * The small, framework-free state core for the emitted HTML player. This file
 * owns two things and nothing else:
 *
 *   1. `CTPS.store` — a single FLAT plain object. The playhead (`t`), transport
 *      (`playing`, `speed`), UI mode (`activeTab`, `fullView`), layer visibility
 *      (`layers`), current `selection`, and the cargo model (`objects`,
 *      `robotCfg`, `scene`, `activeCargoTab`) live directly as top-level
 *      properties ON the store, right next to its action methods. Consumers read
 *      them straight off it (`CTPS.store.t`, `CTPS.store.layers.map`, ...). A
 *      tiny pub/sub lets sibling modules subscribe; every action notifies with
 *      an array of the top-level store keys it touched. There is no computed
 *      state, no batching and no diffing — deliberately a store, not a framework.
 *      The cargo scene is the one exception to "no computed state": a controller
 *      subscribed to `["t"]` bisects the resolve-once scene timeline (built by
 *      `loadLayout` via cargo.js) and re-seats `scene` only when its index
 *      changes — a pure bisect, still no per-frame recomputation.
 *   2. `CTPS.cursor` — a zero-argument factory returning a memoized event-cursor
 *      service. `at(t)` bisects the event array through the pinned seek engine;
 *      `step(prevIdx, idx, callbacks)` turns a cursor move into forward/rebuild
 *      callbacks so one bisect can feed every panel.
 *
 * Pure state only: no DOM access, no rendering, no styling — those live in the
 * sibling render/transport/panel assets. This file is concatenated into its own
 * script element AFTER player.js (so `window.CTPS` and `CTPS.engine` already
 * exist and are fully populated when this IIFE runs) and BEFORE the render and
 * transport assets that consume the store. Vanilla JS only — no frameworks, no
 * network, works offline.
 */

"use strict";

(function () {
    var CTPS = window.CTPS;

    // ----- initial state (flat — fields live ON the store) ----------------
    // Declared as a data literal first; the action methods are attached onto
    // `store` afterwards (as plain function references, never `this`-bound) so
    // later files can destructure or pass those functions around without losing
    // their binding.
    var store = {
        t: 0,
        playing: false,
        speed: 1,
        activeTab: "log",
        fullView: false,
        // Layer visibility mirrors the `.ctps-layer-chips` markup's initial
        // `ctps-chip--active` state (chrome.js's refreshLayerChips reads
        // `store.layers[name]` on init to seat each chip, so a chip whose key is
        // absent here would be forced OFF). The randomized-vision layers
        // `mosaic` / `vision-cone` (chip keys `data-layer="mosaic"` /
        // `data-layer="vision-cone"`) default ON like their active chips —
        // vision.js's layers no-op anyway when no MOSAIC / cone DATA is present.
        layers: { map: true, trail: true, path: true, nodes: true,
                  objects: true, zones: false, deploy: false,
                  mosaic: true, "vision-cone": true },
        selection: { kind: "robot", id: null },
        // ----- cargo model (object/cargo plan -- issue #57) ---------------
        // `objects`/`robotCfg` come from a loaded layout (the `OBJECTS` global or an
        // uploaded file); `scene` is the current resolved cargo snapshot (or
        // null before a layout / before the first manip event); `activeCargoTab`
        // is the right-rail cargo panel's active tab. Visibility for the new
        // objects/zones layers reuses `layers` above (no new toggle action).
        objects: [],
        robotCfg: null,
        scene: null,
        // ----- randomized-vision model (ADR-060) --------------------------
        // `mosaicCells` is the resolved 12-cell mosaic the vision layers/HUD
        // read: the embedded `MOSAIC` global on an emitted page, OR an UPLOADED
        // layout's `mosaic` block re-resolved through vision.js on a mosaic-free
        // page (ADR-056 upload leg). `mosaicMeta` carries the colour vocabulary
        // (`{names, hex}`) so the cell fills read from ONE source; `visionCone`
        // is the robot-LOCAL cone geometry (`robot.vision`). All null/absent
        // until a mosaic or cone is present — the vision layers then no-op.
        mosaicCells: null,
        mosaicMeta: null,
        visionCone: null,
        // `sorterError` is set by `loadLayout` from the resolver: a message when
        // the loaded trace fired a grab with no sorter lane selected, else null.
        sorterError: null,
        activeCargoTab: "magazine",
    };

    // ----- subscription ----------------------------------------------------
    var listeners = [];

    /**
     * Fan a change out to every subscriber. Iterates a snapshot copy of the
     * listener list so a subscriber that unsubscribes itself (or subscribes a
     * new listener) mid-notify cannot corrupt the loop. Each subscriber is
     * called as `fn(changedKeys, store)` — the second argument is the store
     * itself, so a listener can read any current field straight off it.
     *
     * @param changedKeys array of top-level store keys that changed.
     */
    function notify(changedKeys) {
        var snapshot = listeners.slice();
        for (var i = 0; i < snapshot.length; i++) {
            snapshot[i](changedKeys, store);
        }
    }

    /**
     * The track's total duration in seconds, or 0 when no track (or no
     * duration) was ingested. Private helper — not exposed on `store`; used to
     * clamp the playhead in `seek`.
     */
    function duration() {
        return (CTPS.engine.track && CTPS.engine.track.duration_s) || 0;
    }

    store.subscribe = function (fn) {
        listeners.push(fn);
        return function unsubscribe() {
            var idx = listeners.indexOf(fn);
            if (idx !== -1) {
                listeners.splice(idx, 1);
            }
        };
    };

    // ----- actions (every call notifies unconditionally) ------------------

    /**
     * Move the playhead to `t`, clamped into [0, duration()] (low bound applied
     * first, then the high bound), and notify with ["t"].
     */
    store.seek = function (t) {
        var max = duration();
        if (t < 0) {
            t = 0;
        } else if (t > max) {
            t = max;
        }
        store.t = t;
        notify(["t"]);
    };

    // `setT` is a literal alias for `seek` — the same function reference, not a
    // forwarding wrapper.
    store.setT = store.seek;

    store.setPlaying = function (playing) {
        store.playing = !!playing;
        notify(["playing"]);
    };

    store.togglePlay = function () {
        store.setPlaying(!store.playing);
    };

    store.setSpeed = function (speed) {
        store.speed = speed;
        notify(["speed"]);
    };

    // A layer sub-change always reports the single key ["layers"] (never
    // ["layers", name]) — the coarse key is what T4 subscribes on.
    store.setLayer = function (name, visible) {
        store.layers[name] = !!visible;
        notify(["layers"]);
    };

    store.toggleLayer = function (name) {
        store.setLayer(name, !store.layers[name]);
    };

    store.select = function (kind, id) {
        store.selection = { kind: kind, id: (id === undefined) ? null : id };
        notify(["selection"]);
    };

    store.setTab = function (name) {
        store.activeTab = name;
        notify(["activeTab"]);
    };

    store.setFull = function (full) {
        store.fullView = !!full;
        notify(["fullView"]);
    };

    store.toggleFull = function () {
        store.setFull(!store.fullView);
    };

    // ----- cargo scene wiring (object/cargo plan -- issue #57) -------------
    // The resolved scene timeline (one snapshot per manip event) and the index
    // of the snapshot currently shown live as closure state; `store.scene` is
    // the snapshot itself. `loadLayout` runs the resolver (owned by cargo.js,
    // inlined AFTER this file, so it is reached via `CTPS.cargo` at call time);
    // a controller subscribed to ["t"] re-derives the shown snapshot by a pure
    // bisect and only re-notifies when the scene INDEX changes.
    var sceneTimeline = [];
    var sceneIdx = -1;

    /**
     * Index of the last scene snapshot whose time is <= `t`, or -1 if none
     * (playhead before the first manip event). Bounded binary search over the
     * non-decreasing timeline — the same bisect shape as engine.findEventIndex.
     */
    function sceneIndexAt(t) {
        var n = sceneTimeline.length;
        if (n === 0 || t < sceneTimeline[0].t) {
            return -1;
        }
        if (t >= sceneTimeline[n - 1].t) {
            return n - 1;
        }
        var lo = 0;
        var hi = n - 1;
        while (hi - lo > 1) {
            var mid = lo + Math.floor((hi - lo) / 2);
            if (sceneTimeline[mid].t <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    /** The snapshot at timeline index `idx`, or the pristine scene when idx < 0. */
    function sceneAt(idx) {
        if (idx >= 0 && idx < sceneTimeline.length) {
            return sceneTimeline[idx];
        }
        return (CTPS.cargo && CTPS.cargo.emptyScene) ? CTPS.cargo.emptyScene() : null;
    }

    /** Set the current cargo snapshot and notify ["scene"]. */
    function setScene(snapshot) {
        store.scene = snapshot;
        notify(["scene"]);
    }

    /**
     * Seat the randomized-vision embed (ADR-060) on the store: the resolved
     * mosaic cells, the colour-vocabulary meta, and the robot-local vision cone.
     * Called by vision.js on boot with the emitted `MOSAIC`/`MOSAIC_META`
     * globals + the cone from the loaded `robot.vision`, and again by
     * `loadLayout` on the upload leg (an uploaded `mosaic`/`robot.vision` layout
     * re-resolved through vision.js). Any argument may be null (absent feature);
     * the vision layers/HUD then no-op. Notifies the touched keys so the layers
     * repaint and the HUD refreshes.
     *
     * @param cells the resolved 12-cell array (`[{index,x_cm,y_cm,color?}]`), or
     *     null when no mosaic is present.
     * @param meta the `{names, hex}` colour vocabulary, or null.
     * @param cone the robot-local `{origin_x_cm, origin_y_cm, half_angle_deg,
     *     range_cm}` cone, or null.
     */
    store.setVisionEmbed = function (cells, meta, cone) {
        store.mosaicCells = cells || null;
        store.mosaicMeta = meta || null;
        store.visionCone = cone || null;
        notify(["mosaicCells", "mosaicMeta", "visionCone"]);
    };

    /**
     * Ingest a layout dict (`{ objects, robot, mosaic? }` — the embedded
     * `OBJECTS` global or an uploaded file): stash `objects`/`robotCfg`, resolve
     * the scene timeline once via cargo.js, seat the snapshot for the current
     * playhead, hoist the resolver's grab-before-sortTo flag onto `sorterError`,
     * and notify the touched keys in one shot.
     *
     * ADR-056 upload leg (randomized vision): when the dict carries a `mosaic`
     * geometry block or a `robot.vision` cone, re-derive the vision embed through
     * vision.js (`CTPS.vision.embedFromLayout`) and seat it via
     * `setVisionEmbed`, so an uploaded mosaic/cone renders on a page emitted
     * WITHOUT one (the embedded `MOSAIC` path is untouched — vision.js
     * seats that itself on boot). A dict with neither leaves the current embed
     * in place (an upload of a mosaic-free layout does not clobber an emitted
     * mosaic; a fresh emitted page starts null anyway).
     */
    store.loadLayout = function (layoutDict) {
        layoutDict = layoutDict || {};
        store.objects = layoutDict.objects || [];
        store.robotCfg = (layoutDict.robot !== undefined) ? layoutDict.robot : null;
        var resolve = (CTPS.cargo && CTPS.cargo.resolve) || null;
        sceneTimeline = resolve
            ? resolve(CTPS.engine.track, store.objects, store.robotCfg)
            : [];
        store.sorterError = (sceneTimeline && sceneTimeline.sorterError) || null;
        sceneIdx = sceneIndexAt(store.t);
        store.scene = sceneAt(sceneIdx);
        notify(["objects", "robotCfg", "scene", "sorterError"]);

        // Upload leg: re-derive the vision embed from the uploaded layout's own
        // `mosaic`/`robot.vision` blocks (vision.js owns the resolve so the cell
        // geometry stays single-sourced). Only overwrites the embed when the
        // uploaded dict actually carries mosaic geometry or a cone, so uploading
        // a cargo-only layout never wipes an embedded mosaic.
        var embed = (CTPS.vision && CTPS.vision.embedFromLayout)
            ? CTPS.vision.embedFromLayout(layoutDict)
            : null;
        if (embed && (embed.cells || embed.cone)) {
            store.setVisionEmbed(embed.cells, embed.meta, embed.cone);
        }
    };

    store.setCargoTab = function (name) {
        store.activeCargoTab = name;
        notify(["activeCargoTab"]);
    };

    // The scene controller: on every playhead move, bisect the timeline and
    // re-seat the snapshot ONLY when its index changed, so the cargo panel / HUD
    // / cargo timeline lanes refresh on cargo events, not on every frame. Subscribed to
    // ["t"] only; setScene notifies ["scene"], which this controller ignores, so
    // there is no feedback loop.
    store.subscribe(function (changedKeys) {
        if (changedKeys.indexOf("t") === -1) {
            return;
        }
        var idx = sceneIndexAt(store.t);
        if (idx === sceneIdx) {
            return;
        }
        sceneIdx = idx;
        setScene(sceneAt(idx));
    });

    CTPS.store = store;

    // ----- event-cursor service -------------------------------------------
    // Built once and cached: the service holds no per-call state, so every
    // `CTPS.cursor()` call returns this same reference.
    var cursorService = {
        at: function (t) {
            return CTPS.engine.findEventIndex(t);
        },
        step: function (prevIdx, idx, callbacks) {
            callbacks = callbacks || {};
            if (idx > prevIdx) {
                if (callbacks.forward) {
                    callbacks.forward(
                        CTPS.engine.events.slice(prevIdx + 1, idx + 1));
                }
            } else if (idx < prevIdx) {
                if (callbacks.rebuild) {
                    callbacks.rebuild(idx);
                }
            }
            // idx === prevIdx: nothing changed, no callback fires.
        },
    };

    /**
     * Return the memoized event-cursor service (same reference every call).
     * `at(t)` resolves the current event index via the pinned bisect; `step`
     * diffs a previous index against a new one and fires the caller's forward
     * (slice of newly-crossed events) or rebuild (replay-from callback). `step`
     * never calls `at` itself — the caller bisects once per refresh and feeds
     * the resulting index to every consumer's own `step`.
     */
    CTPS.cursor = function () {
        return cursorService;
    };

    // No CTPS._inits push: `store` and `cursorService` are both fully
    // constructed synchronously the moment this IIFE runs — `store`'s fields are
    // literals and `CTPS.engine` is already populated (player.js finished before
    // this file). There is no boot-time / async setup to defer into an init(ctx)
    // callback, so adding a no-op init would be dead code.
})();
