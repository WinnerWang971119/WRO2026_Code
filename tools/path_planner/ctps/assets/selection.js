/* CTPS — Clear Trajectory Prediction Simulator: canvas selection -> inspector.
 *
 * Owns the one interaction that turns a canvas click into a selection, plus
 * the four `inspectorField` descriptors the Inspector tab (panels.js/T7)
 * renders for whichever entity is selected:
 *
 *   1. Click wiring — a click on `#ctps-canvas` (the topmost, actors canvas)
 *      is converted to CSS-pixel coordinates relative to that canvas's own
 *      box, handed to `CTPS.renderer.hitTest(px, py)`, and a hit becomes
 *      `CTPS.store.select(hit.kind, hit.id)`. A miss (`null`) leaves the
 *      current selection alone — the robot stays selected by default
 *      (`store.selection` starts `{kind:"robot", id:null}`, per store.js).
 *   2. `inspectorField` registrations — `pose`, `heading`, `nearest`,
 *      `drift-check`, in that order. Each descriptor returns plain display
 *      text plus, where relevant, a semantic `tone` hint ("drift" or null)
 *      for panels.js to map onto a CSS modifier class; NEVER a colour
 *      literal, so the value/tone split stays the only place that
 *      one-token-value coupling would otherwise creep in.
 *
 * This file renders nothing itself — no DOM writes beyond the click
 * listener's side effect (a store action). The Inspector's actual markup is
 * built by panels.js, which re-renders on every `"t"` / `"selection"` store
 * change and calls each field's `value(state)`/`tone(state)` fresh. Vanilla
 * ES5 (var/function) — no frameworks, no network, works offline.
 *
 * Coordinate convention (matches player.js / renderer.js): centimetres,
 * degrees in [0, 360), 0 deg = +x (East), CCW positive, field origin SW with
 * +Y North. `field.nodes` is the ingested Track's LOWERCASE `{name:[x,y]}`
 * map (Data Model B).
 */

"use strict";

(function () {
    var CTPS = window.CTPS;

    //: "OK" drift-check text below the drift threshold used by track_builder
    //: (DRIFT_THRESHOLD_CM = 10) — literal per the UI spec; this is display
    //: copy, not a computed value, so it is not threaded through Config.
    var DRIFT_OK_TEXT = "OK · < 10 cm";

    // ----- field/node helpers ------------------------------------------------

    /** The ingested track's `field` object, or null when no track/field. */
    function _field() {
        var track = CTPS.engine.track;
        return (track && track.field) || null;
    }

    /** The field's nodes map `{name:[x,y]}` (LOWERCASE key), or {}. */
    function _fieldNodes() {
        var f = _field();
        return (f && f.nodes) || {};
    }

    /** "(x.x, y.y) cm" — the shared pose/coordinate text format. */
    function _formatXY(x, y) {
        return "(" + x.toFixed(1) + ", " + y.toFixed(1) + ") cm";
    }

    /**
     * The world point the inspector should reason about for the current
     * selection: the live interpolated pose for a robot selection, or the
     * selected node's own coordinates for a node selection. Returns null for
     * an unknown selection kind or a node id that is not (or no longer) in
     * `field.nodes`.
     */
    function _selectionPoint(state) {
        var sel = state.selection || {};
        if (sel.kind === "robot") {
            return CTPS.engine.poseAt(state.t);
        }
        if (sel.kind === "node") {
            var coord = _fieldNodes()[sel.id];
            return coord ? { x: coord[0], y: coord[1] } : null;
        }
        return null;
    }

    /**
     * The nearest `field.nodes` entry to world point `(x, y)`, as
     * `{name, d}` (`d` in cm), or null when the field carries no nodes.
     */
    function _nearestNode(x, y) {
        var nodes = _fieldNodes();
        var bestName = null;
        var bestD = 0;
        for (var name in nodes) {
            if (!Object.prototype.hasOwnProperty.call(nodes, name)) {
                continue;
            }
            var c = nodes[name];
            var d = Math.hypot(x - c[0], y - c[1]);
            if (bestName === null || d < bestD) {
                bestName = name;
                bestD = d;
            }
        }
        return bestName === null ? null : { name: bestName, d: bestD };
    }

    // ----- drift-check: last drift_warn event at-or-before t ----------------

    /**
     * Parse a `drift_warn` label of the exact form
     * `"chassis 12.4 cm off node J6"` (track_builder.py's format) into
     * `{cm, node}`, or null when the label does not match.
     */
    function _parseDriftLabel(label) {
        if (!label) {
            return null;
        }
        var match = /([0-9]*\.?[0-9]+)\s*cm off node\s+(\S+)/.exec(label);
        if (!match) {
            return null;
        }
        return { cm: parseFloat(match[1]), node: match[2] };
    }

    // Just the `drift_warn` events, filtered out of `CTPS.engine.events`
    // once on first read and reused. It stays time-sorted because the source
    // `events` array is, which is what lets `_driftCheckAt` bisect it in
    // O(log D) rather than scan the full event prefix every frame. The extra
    // one-slot memo keyed on the last-seen `t` lets the Inspector's
    // `value(state)` and `tone(state)` share a single lookup per frame —
    // panels.js calls them back-to-back with the same playhead (~60/s).
    var _driftEvents = null;
    var _driftMemoT = null;
    var _driftMemo = null;

    /** The cached `drift_warn`-only slice of `CTPS.engine.events`. */
    function _getDriftEvents() {
        if (_driftEvents === null) {
            _driftEvents = [];
            var events = CTPS.engine.events || [];
            for (var i = 0; i < events.length; i += 1) {
                if (events[i].type === "drift_warn") {
                    _driftEvents.push(events[i]);
                }
            }
        }
        return _driftEvents;
    }

    /**
     * The most recent `drift_warn` event at-or-before `t`, parsed, or null
     * when none exists yet. Binary-searches the cached `drift_warn`-only
     * array (time-sorted, since `events` is) for the last entry with
     * `t <= playhead` — O(log D) in the drift-event count and independent of
     * how many non-drift events precede it, so the common clean mission (zero
     * `drift_warn`) resolves in O(1). Mirrors `findEventIndex`'s bisect.
     */
    function _driftCheckAt(t) {
        if (t === _driftMemoT) {
            return _driftMemo;
        }
        var driftEvents = _getDriftEvents();
        var n = driftEvents.length;
        var result = null;
        if (n > 0 && t >= driftEvents[0].t) {
            var idx;
            if (t >= driftEvents[n - 1].t) {
                idx = n - 1;
            } else {
                var lo = 0;
                var hi = n - 1;
                while (hi - lo > 1) {
                    var mid = lo + Math.floor((hi - lo) / 2);
                    if (driftEvents[mid].t <= t) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                idx = lo;
            }
            result = _parseDriftLabel(driftEvents[idx].label);
        }
        _driftMemoT = t;
        _driftMemo = result;
        return result;
    }

    // ----- inspectorField registrations (pose, heading, nearest, drift-check) -

    CTPS.inspectorFields.register({
        id: "pose",
        label: "pose",
        value: function (state) {
            var sel = state.selection || {};
            if (sel.kind === "robot") {
                var pose = CTPS.engine.poseAt(state.t);
                return _formatXY(pose.x, pose.y);
            }
            if (sel.kind === "node") {
                var coord = _fieldNodes()[sel.id];
                return coord ? _formatXY(coord[0], coord[1]) : "—";
            }
            return "—";
        },
    });

    CTPS.inspectorFields.register({
        id: "heading",
        label: "heading",
        value: function (state) {
            var sel = state.selection || {};
            if (sel.kind !== "robot") {
                return "—";
            }
            var pose = CTPS.engine.poseAt(state.t);
            return pose.h.toFixed(1) + "°";
        },
    });

    CTPS.inspectorFields.register({
        id: "nearest",
        label: "nearest",
        value: function (state) {
            var point = _selectionPoint(state);
            if (!point) {
                return "—";
            }
            var nearest = _nearestNode(point.x, point.y);
            if (!nearest) {
                return "—";
            }
            return nearest.name + " (" + nearest.d.toFixed(1) + " cm)";
        },
    });

    CTPS.inspectorFields.register({
        id: "drift-check",
        label: "drift chk",
        value: function (state) {
            var drift = _driftCheckAt(state.t);
            return drift ? (drift.cm.toFixed(1) + " cm off " + drift.node) : DRIFT_OK_TEXT;
        },
        tone: function (state) {
            return _driftCheckAt(state.t) ? "drift" : null;
        },
    });

    // ----- canvas click -> selection -----------------------------------------

    /**
     * Convert a click on `canvas` to CSS-pixel coordinates relative to that
     * canvas's own box (matching the renderer's CSS-px world<->pixel
     * transform, not backing-store/device pixels), hit-test it, and select
     * the hit. A miss leaves the current selection untouched.
     */
    function _handleClick(canvas, evt) {
        if (!CTPS.renderer || typeof CTPS.renderer.hitTest !== "function") {
            return;
        }
        var rect = canvas.getBoundingClientRect();
        var px = evt.clientX - rect.left;
        var py = evt.clientY - rect.top;
        var hit = CTPS.renderer.hitTest(px, py);
        if (!hit) {
            return;
        }
        CTPS.store.select(hit.kind, hit.id);
    }

    /**
     * Boot hook — wires the click listener once the actors canvas exists in
     * the DOM. A missing canvas is recorded into `CTPS._errors` rather than
     * thrown, matching every sibling module's "one bad module can't take
     * down the page" posture.
     */
    function init(ctx) {
        var canvas = document.getElementById("ctps-canvas");
        if (!canvas) {
            CTPS._errors.push("selection: #ctps-canvas not found");
            return;
        }
        canvas.addEventListener("click", function (evt) {
            _handleClick(canvas, evt);
        });
    }

    CTPS._inits.push(init);
})();
