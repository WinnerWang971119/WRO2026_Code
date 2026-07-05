/* CTPS — Clear Trajectory Prediction Simulator: player store.
 *
 * The small, framework-free state core for the emitted HTML player. This file
 * owns two things and nothing else:
 *
 *   1. `CTPS.store` — a single FLAT plain object. The playhead (`t`), transport
 *      (`playing`, `speed`), UI mode (`activeTab`, `fullView`), layer visibility
 *      (`layers`) and current `selection` live directly as top-level properties
 *      ON the store, right next to its action methods. Consumers read them
 *      straight off it (`CTPS.store.t`, `CTPS.store.layers.map`, ...). A tiny
 *      pub/sub lets sibling modules subscribe; every action notifies with an
 *      array of the top-level store keys it touched. There is no computed state,
 *      no batching and no diffing — deliberately a store, not a framework.
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
        layers: { map: true, trail: true, path: true, nodes: true },
        selection: { kind: "robot", id: null },
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
