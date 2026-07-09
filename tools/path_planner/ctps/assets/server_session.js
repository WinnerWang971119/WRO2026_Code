/* CTPS -- self-closing-server liveness session.
 *
 * SINGLE SOURCE of the edit-server heartbeat/close protocol (ADR-057), shared by
 * the served player and the served layout editor so the interval, endpoints, and
 * beacon strategy can never desync between the two:
 *   - a `POST /ping` every few seconds is the heartbeat; its silence trips the
 *     server's idle watchdog, which is how a quit/crash/sleep gets cleaned up;
 *   - a `/close` beacon on page unload is the clean-close fast path.
 *
 * Self-guarded to the `file:` protocol -- inert on a double-clicked offline page
 * (no server to talk to), so the same bytes work offline. Consumed two ways:
 *   - the editor inlines this file as an ordinary asset (layout_editor_html.py
 *     `_ASSET_JS`), present but inert until served;
 *   - the player injects this file's text into its SERVED copy only
 *     (__main__.py `_inject_player_heartbeat`), keeping the on-disk player file
 *     byte-clean.
 *
 * Same-origin RELATIVE URLs only -- no absolute scheme literal -- so it never
 * trips either page's self-containment lint. Vanilla ES5, no dependencies.
 */
(function () {
    if (!window.location || window.location.protocol === "file:") { return; }
    var HEARTBEAT_MS = 3000;
    setInterval(function () {
        try { fetch("/ping", { method: "POST" }).catch(function () {}); }
        catch (e) { /* server already closed -- fine */ }
    }, HEARTBEAT_MS);
    function closeBeacon() {
        try {
            if (navigator.sendBeacon) { navigator.sendBeacon("/close", ""); }
            else { fetch("/close", { method: "POST", keepalive: true }); }
        } catch (e) { /* no-op */ }
    }
    window.addEventListener("pagehide", closeBeacon);
    window.addEventListener("beforeunload", closeBeacon);
})();
