/* CTPS — Clear Trajectory Prediction Simulator: boot.
 *
 * The bootstrap for the emitted HTML player. This file is the LAST entry in
 * `_ASSET_JS` (see player_html.py Contracts → Emitter) — every other module's
 * top-level IIFE has already run by the time this one does, so `CTPS._inits`
 * and every registry (`layers`, `renderModes`, `timelineLanes`, `panelTabs`,
 * `inspectorFields`) are fully populated. Boot owns exactly the generic
 * startup sequence and nothing feature-specific:
 *
 *   1. Read the inline `TRACK` / `PLAYER_META` globals (guarding `undefined`)
 *      and build the shared `ctx = { store, dom, track, meta }` handed to
 *      every module's `init(ctx)`.
 *   2. Run every `fn` pushed onto `CTPS._inits`, in array (load) order, each
 *      isolated in its own try/catch — one throwing init logs `console.error`
 *      but never aborts the rest (each JS asset is its own <script>, so a bad
 *      module must not be able to take down sibling modules that already ran
 *      or the boot steps that come after it).
 *   3. Validate `PLAYER_META`'s manifests (`lanes`, `tabs`, `render_mode`)
 *      against the registries: an id nobody registered is a soft config
 *      drift — `console.warn`, never a throw (mirrors the existing "unknown
 *      drive.method" 3-place-contract convention).
 *   4. Surface any accumulated `CTPS._errors` (the duplicate-registry-id
 *      channel) via `console.error`.
 *   5. Seek to 0 so every store-subscribed region paints its initial frame.
 *
 * Boot does not know about individual modules or features — it only runs
 * `_inits`, validates manifests, and seeks 0. Resize is owned entirely by the
 * renderer (its own debounced ResizeObserver + window listener), so boot does
 * not wire it. Vanilla JS only — no frameworks, no network, works offline.
 */

"use strict";

(function () {
    var CTPS = window.CTPS;

    /**
     * Warn (never throw) for every id in `ids` that has no matching
     * descriptor in `registry`. A manifest entry naming an id nobody
     * registered is a soft config drift, not a boot-time fatal — the
     * player must keep running with everything else that IS wired.
     *
     * @param ids array of ids from a PLAYER_META manifest (may be falsy).
     * @param registry a CTPS.* descriptor registry (exposes `get(id)`).
     * @param kind human-readable registry kind, used in the warning text.
     */
    function validateManifest(ids, registry, kind) {
        if (!ids) {
            return;
        }
        for (var i = 0; i < ids.length; i++) {
            var id = ids[i];
            if (!registry.get(id)) {
                console.warn("CTPS: no " + kind + " registered for '" + id + "'");
            }
        }
    }

    /**
     * Run the full boot sequence once, in the order documented above.
     */
    function boot() {
        var track = (typeof TRACK !== "undefined") ? TRACK : null;
        var meta = (typeof PLAYER_META !== "undefined") ? PLAYER_META : {};
        var ctx = {
            store: CTPS.store,
            dom: document,
            track: track,
            meta: meta,
        };

        // ----- 1/2: run every init, in load order, isolating failures -----
        var inits = CTPS._inits || [];
        for (var i = 0; i < inits.length; i++) {
            try {
                inits[i](ctx);
            } catch (e) {
                console.error("CTPS: init failed", e);
            }
        }

        // ----- 3: validate PLAYER_META manifests against the registries ---
        validateManifest(meta.lanes, CTPS.timelineLanes, "lane");
        validateManifest(meta.tabs, CTPS.panelTabs, "tab");
        if (meta.render_mode !== undefined && meta.render_mode !== null &&
            !CTPS.renderModes.get(meta.render_mode)) {
            console.warn(
                "CTPS: no render mode registered for '" + meta.render_mode + "'");
        }

        // ----- 4: surface accumulated registry errors ----------------------
        var errors = CTPS._errors || [];
        for (var j = 0; j < errors.length; j++) {
            console.error("CTPS: " + errors[j]);
        }

        // ----- 5: seek to 0 so every subscribed region paints its initial
        // frame ---------------------------------------------------------
        if (CTPS.store && CTPS.store.seek) {
            CTPS.store.seek(0);
        }
    }

    if (!CTPS) {
        // player.js creates window.CTPS unconditionally; this should be
        // unreachable, but boot must not throw an uncaught error if it is.
        console.error("CTPS: window.CTPS is missing; boot aborted");
        return;
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", boot);
    } else {
        boot();
    }
})();
