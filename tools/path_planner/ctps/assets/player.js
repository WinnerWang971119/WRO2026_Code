/* CTPS — Clear Trajectory Prediction Simulator: player core.
 *
 * Slim runtime core for the HTML emitted by player_html.render(). This file
 * owns two things and nothing else:
 *
 *   1. Track ingestion + the pinned seek engine. Reads the inline `TRACK`
 *      constant (Data Model B) and exposes findKeyframeIndex / poseAt /
 *      findEventIndex / wrapDeltaDeg — the bounded bisect that every stateful
 *      region resolves through. That bisect is the load-bearing O(log N) seek
 *      path required by AC3a / TC12; its declarations and 4-space indentation
 *      are PINNED — do not relocate or reindent them (TC12 isolates poseAt via
 *      a regex that ends on a 4-space-indented closing brace).
 *   2. The `window.CTPS` namespace. Engine handles plus five descriptor
 *      registries (layers / renderModes / timelineLanes / panelTabs /
 *      inspectorFields) and the boot hooks (_inits / _errors). The namespace
 *      and engine are created UNCONDITIONALLY (even when TRACK is undefined) so
 *      sibling modules always find them. Sibling assets — store.js, renderer.js,
 *      transport.js, timeline.js, panels.js, selection.js, chrome.js, boot.js —
 *      register descriptors and push init callbacks here; boot.js runs _inits in
 *      load order and surfaces _errors.
 *
 * Rendering, transport, narration and canvas code live in the sibling assets,
 * not here. Styling is set only via CSS custom properties in those modules — no
 * JS inline styles (the ADR-053 carve-out). Vanilla JS only — no frameworks, no
 * network, works offline.
 *
 * Coordinate convention (matches tools/path_planner): centimetres, degrees in
 * [0, 360), 0 deg = +x (East), CCW positive. Field origin (0,0) is the SW
 * corner with Y pointing North/up.
 */

"use strict";

(function () {
    // ----- track ingestion (PINNED — engine closes over these) -----------
    // Byte-stable expressions: `track` is null when TRACK is absent, and
    // `keyframes`/`events` degrade to empty arrays so the seek engine never
    // dereferences a missing field. The engine functions below close over
    // `keyframes`/`events`; CTPS.engine re-exposes all three so sibling assets
    // reuse the same arrays.
    var track = (typeof TRACK!=='undefined')?TRACK:null;
    var keyframes = track&&track.keyframes||[];
    var events = track&&track.events||[];

    // =====================================================================
    // Seek engine — AC3a guarantee (PINNED for TC12).
    //
    // findKeyframeIndex(t) is a BOUNDED binary search (bisect) over the baked
    // keyframe array: each step halves [lo, hi], so it runs in O(log N) and
    // touches at most ~log2(N) entries — it does NOT scan the full array. The
    // loop bound is the keyframe count, never the playhead's frame count.
    // poseAt(t) uses that index to lerp between the two bracketing keyframes
    // (x/y linearly, heading on the shortest arc). This is the load-bearing
    // O(1)-class seek path required by AC3a / TC12. Do NOT relocate or reindent
    // findKeyframeIndex / poseAt / findEventIndex / wrapDeltaDeg.
    // =====================================================================

    /**
     * Return the index `i` of the last keyframe whose time is <= `t`
     * (the left element of the bracketing pair). Bounded binary search:
     * the search window strictly shrinks each iteration, capped by the
     * keyframe count. Returns 0 for an empty/degenerate track or a `t`
     * before the first keyframe; returns the last index for a `t` at or
     * past the final keyframe.
     */
    function findKeyframeIndex(t) {
        var n = keyframes.length;
        if (n === 0) {
            return 0;
        }
        if (t <= keyframes[0].t) {
            return 0;
        }
        if (t >= keyframes[n - 1].t) {
            return n - 1;
        }
        var lo = 0;
        var hi = n - 1;
        // Invariant: keyframes[lo].t <= t < keyframes[hi].t. Each step halves
        // the window via Math.floor of the midpoint — bounded by log2(n).
        while (hi - lo > 1) {
            var mid = lo + Math.floor((hi - lo) / 2);
            if (keyframes[mid].t <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    /**
     * Wrap a heading delta (deg) into (-180, +180] so the marker rotates over
     * the shortest arc instead of spinning the long way round.
     */
    function wrapDeltaDeg(d) {
        var x = d % 360.0;
        if (x <= -180.0) {
            x += 360.0;
        } else if (x > 180.0) {
            x -= 360.0;
        }
        return x;
    }

    /**
     * Resolve the interpolated pose {x, y, h} at time `t`. Uses
     * findKeyframeIndex (bounded binary search) to bracket `t`, then lerps:
     * x/y linearly and heading on the shortest arc. Guards the lerp against
     * 0- or 1-keyframe tracks and against a zero-length time bracket.
     */
    function poseAt(t) {
        var n = keyframes.length;
        if (n === 0) {
            return { x: 0, y: 0, h: 0 };
        }
        if (n === 1) {
            return { x: keyframes[0].x, y: keyframes[0].y, h: keyframes[0].h };
        }
        var i = findKeyframeIndex(t);
        if (i >= n - 1) {
            return {
                x: keyframes[n - 1].x,
                y: keyframes[n - 1].y,
                h: keyframes[n - 1].h,
            };
        }
        var a = keyframes[i];
        var b = keyframes[i + 1];
        var span = b.t - a.t;
        var u = (span > 0) ? (t - a.t) / span : 0.0;
        if (u < 0) { u = 0; }
        if (u > 1) { u = 1; }
        return {
            x: a.x + (b.x - a.x) * u,
            y: a.y + (b.y - a.y) * u,
            h: a.h + wrapDeltaDeg(b.h - a.h) * u,
        };
    }

    /**
     * Index of the last event whose time is <= `t`, or -1 if none. Bounded
     * binary search over the (non-decreasing) event array — same bounded
     * pattern as the keyframe lookup.
     */
    function findEventIndex(t) {
        var n = events.length;
        if (n === 0 || t < events[0].t) {
            return -1;
        }
        if (t >= events[n - 1].t) {
            return n - 1;
        }
        var lo = 0;
        var hi = n - 1;
        while (hi - lo > 1) {
            var mid = lo + Math.floor((hi - lo) / 2);
            if (events[mid].t <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    // ----- descriptor registries ------------------------------------------
    // _inits and _errors are shared arrays: the registries push duplicate ids
    // into the very array CTPS._errors points at, and modules push init
    // callbacks onto the array CTPS._inits points at.
    var _inits = [];
    var _errors = [];

    /**
     * Build a descriptor registry. A registry records descriptors keyed by
     * their `id` and offers ordered iteration. A duplicate `id` is NON-FATAL:
     * it is recorded into the shared `_errors` array (surfaced by boot via
     * console.error) and the duplicate is dropped — register never throws, so
     * one bad module can't abort the page (each asset is its own <script>).
     *
     * @param kind human-readable registry kind, used in the duplicate message.
     */
    function makeRegistry(kind) {
        var items = [];
        var byId = {};
        return {
            register: function (descriptor) {
                var id = descriptor && descriptor.id;
                if (id !== undefined && id !== null &&
                    Object.prototype.hasOwnProperty.call(byId, id)) {
                    _errors.push("duplicate " + kind + " id: " + String(id));
                    return;
                }
                byId[id] = descriptor;
                items.push(descriptor);
            },
            all: function () {
                return items.slice();
            },
            forEach: function (fn) {
                for (var i = 0; i < items.length; i++) {
                    fn(items[i], i);
                }
            },
            get: function (id) {
                return Object.prototype.hasOwnProperty.call(byId, id)
                    ? byId[id]
                    : null;
            },
        };
    }

    var layers = makeRegistry("layer");
    var renderModes = makeRegistry("renderMode");
    var timelineLanes = makeRegistry("timelineLane");
    var panelTabs = makeRegistry("panelTab");
    var inspectorFields = makeRegistry("inspectorField");

    // ----- namespace (created UNCONDITIONALLY) ----------------------------
    // Even when TRACK is undefined, CTPS + engine exist so boot/modules always
    // find the namespace. `store` (T3) and `renderer` (T4) are filled in later.
    window.CTPS = {
        engine: {
            findKeyframeIndex: findKeyframeIndex,
            poseAt: poseAt,
            findEventIndex: findEventIndex,
            wrapDeltaDeg: wrapDeltaDeg,
            track: track,
            keyframes: keyframes,
            events: events,
        },
        layers: layers,
        renderModes: renderModes,
        timelineLanes: timelineLanes,
        panelTabs: panelTabs,
        inspectorFields: inspectorFields,
        store: null,
        renderer: null,
        _inits: _inits,
        _errors: _errors,
    };
})();
