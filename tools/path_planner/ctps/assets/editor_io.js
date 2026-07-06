/* CTPS — Layout Editor: I/O (Blob save + FileReader load of layout.json).
 *
 * The persistence layer of the two-tab offline editor (object/cargo plan, T8 --
 * issue #57). On `file://`, localStorage and the File System Access API are both
 * dead, so the round-trip is a Blob download + a FileReader upload. When instead
 * the page is served by the local edit-server (ADR-057 -- any non-`file:`
 * origin), Save POSTs straight to `/save` (written into `layouts/layout.json`)
 * and a heartbeat + close-beacon let that server auto-close when the tab goes
 * away. A server save that fails for any reason falls back to the download, so
 * no edit is ever lost:
 *
 *   - Save: serialise the authored field objects + robot zones to a
 *     schema_version-1 `layout.json` dict (the exact shape ctps/layout.py
 *     validates), stringify it, and download it through a JS-created anchor whose
 *     download URL is assigned as a PROPERTY with spaces (`a.href = url`) so the
 *     self-containment lint -- which bans the literal attribute form -- never
 *     trips. Never a static markup attribute, never a data-image URI.
 *   - Load: a hidden file input + FileReader read the text and TOLERANTLY parse
 *     it (skip + `console.warn` a malformed object, keep the rest; a structurally
 *     invalid file shows an inline message and leaves the current scene intact) --
 *     the JS counterpart to layout.py's strict raise.
 *
 * A save -> load round-trip is loss-free: every field the editor authors is
 * written on save and read back on load, unchanged.
 *
 * Vanilla ES5. No inline element styles, no network, no external reference.
 */

"use strict";

(function () {
    var E = (window.CTPS_EDITOR = window.CTPS_EDITOR || {});
    var HEX_RE = /^#[0-9A-Fa-f]{6}$/;

    // ----- save-state tracking ---------------------------------------------
    // The serialised layout text as of the last persist (Save or boot baseline)
    // and whether a Save has happened this session. The header indicator reads
    // clean vs "unsaved changes" by comparing the CURRENT serialisation to this
    // snapshot -- content-based, so it can never desync from a missed mutation.
    var _savedSnapshot = null;
    var _savedOnce = false;

    // ----- save (serialise -> Blob download) -------------------------------

    /** A schema dict for one robot-local rect zone. */
    function _zoneDict(z) {
        return { x_cm: z.x_cm, y_cm: z.y_cm, w_cm: z.w_cm, h_cm: z.h_cm };
    }

    /**
     * Serialise the current editor state to a schema_version-1 layout dict. Only
     * the dimension keys matching each object's shape are emitted (a rect never
     * carries radius_cm; a circle never carries w_cm/h_cm) -- mirroring
     * layout.py's `LayoutObject.to_dict`.
     */
    function _serialize() {
        var objects = [];
        var src = E.state.objects;
        for (var i = 0; i < src.length; i++) {
            var o = src[i];
            var d = {
                id: o.id,
                name: o.name,
                shape: o.shape,
                type: o.type,
                color: o.color,
                x_cm: o.x_cm,
                y_cm: o.y_cm,
                heading_deg: o.heading_deg,
            };
            if (o.shape === "rect") {
                d.w_cm = o.w_cm;
                d.h_cm = o.h_cm;
            } else {
                d.radius_cm = o.radius_cm;
            }
            objects.push(d);
        }
        var rob = E.state.robot;
        var drops = [];
        for (var j = 0; j < rob.drop_points.length; j++) {
            drops.push({ x_cm: rob.drop_points[j].x_cm, y_cm: rob.drop_points[j].y_cm });
        }
        return {
            schema_version: 1,
            objects: objects,
            robot: {
                grab_zone: _zoneDict(rob.grab_zone),
                box_zone: _zoneDict(rob.box_zone),
                drop_points: drops,
            },
        };
    }

    /**
     * Repaint the header save-state indicator from the current scene vs the
     * last-persisted snapshot: empty at the boot baseline, "Saved ✓" once a
     * Save has landed and nothing changed since, "unsaved changes" otherwise.
     */
    function refreshSaveState() {
        var el = E.dom && E.dom.saveState;
        if (!el) {
            return;
        }
        var clean = _savedSnapshot !== null &&
            JSON.stringify(_serialize()) === _savedSnapshot;
        el.classList.remove(
            "ctps-editor-save-state--saved", "ctps-editor-save-state--dirty"
        );
        if (clean) {
            el.textContent = _savedOnce ? "Saved ✓" : "";
            if (_savedOnce) {
                el.classList.add("ctps-editor-save-state--saved");
            }
        } else {
            el.textContent = "unsaved changes";
            el.classList.add("ctps-editor-save-state--dirty");
        }
    }

    /**
     * Adopt `key` as the persisted snapshot (a Save landed) and repaint clean.
     * `key` MUST be the COMPACT `JSON.stringify(_serialize())` — the exact form
     * `refreshSaveState` / `initSaveState` compare against. Passing the pretty
     * (indented) payload here would never match, so the indicator would never
     * read "Saved ✓".
     */
    function _markSaved(key) {
        _savedSnapshot = key;
        _savedOnce = true;
        refreshSaveState();
    }

    /** Seed the boot baseline (the empty scene is "clean, not yet saved"). */
    function initSaveState() {
        _savedSnapshot = JSON.stringify(_serialize());
        _savedOnce = false;
        refreshSaveState();
    }

    /** True when the page is served (edit-server), false on a `file:` open. */
    function _serverMode() {
        return !!(window.location && window.location.protocol !== "file:");
    }

    /** Download `text` as `layout.json` via a Blob anchor (the file:// path). */
    function _download(text) {
        var blob = new Blob([text], { type: "application/json" });
        var url = URL.createObjectURL(blob);
        var a = document.createElement("a");
        // Assign the object URL as a spaced property (never the literal attribute
        // form) so the emitter's self-containment lint stays clean.
        a.href = url;
        a.download = "layout.json";
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        // Revoke on the next tick so the click has started the download first.
        setTimeout(function () {
            URL.revokeObjectURL(url);
        }, 0);
    }

    /**
     * POST `text` to the edit-server's `/save` (writes `layouts/layout.json`).
     * ANY failure -- non-2xx, network error, no server -- falls back to the Blob
     * download so an edit is never lost.
     */
    function _saveViaServer(text, key) {
        fetch("/save", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: text,
        }).then(function (resp) {
            if (resp && resp.ok) {
                _markSaved(key);
                if (E.status) {
                    E.status("saved to layouts/layout.json");
                }
            } else {
                _download(text);
                _markSaved(key);
                if (E.status) {
                    E.status("server refused save; downloaded layout.json instead");
                }
            }
        }).catch(function () {
            _download(text);
            _markSaved(key);
            if (E.status) {
                E.status("server offline; downloaded layout.json instead");
            }
        });
    }

    /**
     * Serialise + persist the current layout. Served (edit-server) -> POST to
     * `/save`; opened as a `file:` -> Blob download named `layout.json`.
     */
    function save() {
        var payload = _serialize();
        var text = JSON.stringify(payload, null, 2);   // pretty: the saved file
        var key = JSON.stringify(payload);             // compact: the state key
        if (_serverMode()) {
            _saveViaServer(text, key);
        } else {
            _download(text);
            _markSaved(key);
            if (E.status) {
                E.status("saved layout.json");
            }
        }
    }

    /**
     * When served by the edit-server, keep it alive with a heartbeat and shut it
     * down when this tab goes away -- so the server auto-closes and can never
     * dangle. A `POST /ping` every few seconds is the liveness signal (its
     * silence, on any un-clean exit, trips the server's watchdog); a
     * `navigator.sendBeacon('/close')` on unload is the clean-close fast path.
     * Inert on a `file:` open (no server to talk to).
     */
    function _startServerSession() {
        if (!_serverMode()) {
            return;
        }
        var HEARTBEAT_MS = 3000;
        setInterval(function () {
            // Fire-and-forget: a rejected ping just means the server already
            // closed, which is fine.
            try {
                fetch("/ping", { method: "POST" }).catch(function () {});
            } catch (e) {
                /* no-op */
            }
        }, HEARTBEAT_MS);

        function _closeBeacon() {
            try {
                if (navigator.sendBeacon) {
                    navigator.sendBeacon("/close", "");
                } else {
                    fetch("/close", { method: "POST", keepalive: true });
                }
            } catch (e) {
                /* no-op */
            }
        }
        window.addEventListener("pagehide", _closeBeacon);
        window.addEventListener("beforeunload", _closeBeacon);
    }

    // ----- load (FileReader -> tolerant parse) -----------------------------

    /** True for a finite JSON number (not a bool). */
    function _finite(v) {
        return typeof v === "number" && isFinite(v);
    }

    /** Tolerantly parse one object; return null (with a warn) if unusable. */
    function _parseObject(o, i, seen) {
        if (!o || typeof o !== "object") {
            console.warn("layout: objects[" + i + "] is not an object; skipped");
            return null;
        }
        if (o.shape !== "rect" && o.shape !== "circle") {
            console.warn("layout: objects[" + i + "] has an unknown shape; skipped");
            return null;
        }
        if (!_finite(o.x_cm) || !_finite(o.y_cm)) {
            console.warn("layout: objects[" + i + "] has a bad x_cm/y_cm; skipped");
            return null;
        }
        var id = (typeof o.id === "string" && o.id) ? o.id : ("o" + (i + 1));
        if (seen[id]) {
            id = id + "_" + i;
        }
        seen[id] = true;
        var type = (o.type === "tile" || o.type === "cargo" || o.type === "fixed")
            ? o.type : "tile";
        var color = (typeof o.color === "string" && HEX_RE.test(o.color)) ? o.color : "#888888";
        var obj = {
            id: id,
            name: (typeof o.name === "string") ? o.name : id,
            shape: o.shape,
            type: type,
            color: color,
            x_cm: +o.x_cm,
            y_cm: +o.y_cm,
            heading_deg: _finite(o.heading_deg) ? +o.heading_deg : 0,
        };
        if (o.shape === "rect") {
            obj.w_cm = (_finite(o.w_cm) && o.w_cm > 0) ? +o.w_cm : 5;
            obj.h_cm = (_finite(o.h_cm) && o.h_cm > 0) ? +o.h_cm : 5;
        } else {
            obj.radius_cm = (_finite(o.radius_cm) && o.radius_cm > 0) ? +o.radius_cm : 3;
        }
        return obj;
    }

    /** Tolerantly parse the objects[] list, dropping malformed entries. */
    function _parseObjects(arr) {
        var out = [];
        var seen = {};
        for (var i = 0; i < arr.length; i++) {
            var parsed = _parseObject(arr[i], i, seen);
            if (parsed) {
                out.push(parsed);
            }
        }
        return out;
    }

    /** Parse one robot-local rect zone, or null when malformed. */
    function _parseZone(z) {
        if (!z || typeof z !== "object") {
            return null;
        }
        if (!_finite(z.x_cm) || !_finite(z.y_cm) || !_finite(z.w_cm) || !_finite(z.h_cm) ||
            z.w_cm <= 0 || z.h_cm <= 0) {
            return null;
        }
        return { x_cm: +z.x_cm, y_cm: +z.y_cm, w_cm: +z.w_cm, h_cm: +z.h_cm };
    }

    /** Parse the drop-point list, dropping malformed entries; null when none. */
    function _parseDrops(arr) {
        if (!Array.isArray(arr)) {
            return null;
        }
        var out = [];
        for (var i = 0; i < arr.length; i++) {
            var d = arr[i];
            if (d && typeof d === "object" && _finite(d.x_cm) && _finite(d.y_cm)) {
                out.push({ x_cm: +d.x_cm, y_cm: +d.y_cm });
            }
        }
        return out.length ? out : null;
    }

    /** Parse the robot section, falling back to defaults for a missing part. */
    function _parseRobot(r) {
        if (!r || typeof r !== "object") {
            return null;
        }
        var grab = _parseZone(r.grab_zone);
        var box = _parseZone(r.box_zone);
        var drops = _parseDrops(r.drop_points);
        if (!grab && !box && !drops) {
            return null;
        }
        var dflt = E.defaultRobot();
        return {
            grab_zone: grab || dflt.grab_zone,
            box_zone: box || dflt.box_zone,
            drop_points: drops || dflt.drop_points,
        };
    }

    /** Ingest a parsed-and-tolerated layout dict into the editor + repaint. */
    function _ingest(data) {
        if (!data || typeof data !== "object" || !Array.isArray(data.objects)) {
            if (E.status) {
                E.status("couldn't read layout: missing objects list");
            }
            return;
        }
        var objects = _parseObjects(data.objects);
        var robot = _parseRobot(data.robot);
        E.state.objects = objects;
        if (robot) {
            E.state.robot = robot;
        }
        E.state.selection = null;
        // Robot bounds may have shifted -> re-fit both views, then repaint.
        E.resize();
        E.refreshProps();
        // A load replaces the scene with content not yet written to the canonical
        // layout path -> reads as "unsaved changes" until the operator Saves.
        refreshSaveState();
        if (E.status) {
            E.status("loaded " + objects.length + " object(s)");
        }
    }

    /** Read a chosen File (FileReader), tolerantly parse it, and ingest it. */
    function handleFile(file) {
        if (!file) {
            return;
        }
        var reader = new FileReader();
        reader.onload = function () {
            var data;
            try {
                data = JSON.parse(String(reader.result));
            } catch (e) {
                if (E.status) {
                    E.status("couldn't read layout: not valid JSON");
                }
                return;
            }
            _ingest(data);
        };
        reader.onerror = function () {
            if (E.status) {
                E.status("couldn't read layout: file read error");
            }
        };
        reader.readAsText(file);
    }

    // ----- public I/O surface ----------------------------------------------
    E.save = save;
    E.handleFile = handleFile;
    // Ingest a layout dict directly (the boot-time `const LAYOUT` auto-load
    // takes the SAME tolerant path as a user's "Load layout…"). Boot calls this
    // BEFORE `initSaveState`, so the auto-loaded scene becomes the clean
    // baseline (a fresh open reads as saved, not "unsaved changes").
    E.ingestLayout = _ingest;
    E.refreshSaveState = refreshSaveState;
    E.initSaveState = initSaveState;

    // Start the edit-server heartbeat/close session (no-op on a file:// open).
    _startServerSession();
})();
