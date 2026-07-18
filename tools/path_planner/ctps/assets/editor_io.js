/* CTPS — Layout Editor: I/O (Blob save + FileReader load of layout.json).
 *
 * The persistence layer of the two-tab offline editor (object/cargo plan, T8 --
 * issue #57; randomized-vision plan, T16 -- ADR-060). On `file://`, localStorage
 * and the File System Access API are both dead, so the round-trip is a Blob
 * download + a FileReader upload. When instead the page is served by the local
 * edit-server (ADR-057 -- any non-`file:` origin), Save POSTs straight to
 * `/save` (written into `layouts/layout.json`) and a heartbeat + close-beacon
 * let that server auto-close when the tab goes away. A server save that fails
 * for any reason falls back to the download, so no edit is ever lost:
 *
 *   - Save: serialise the authored field objects + robot zones + (T16) the
 *     `mosaic` block + `robot.vision` cone to a schema_version-1 `layout.json`
 *     dict (the exact shape ctps/layout.py validates), stringify it, and
 *     download it through a JS-created anchor whose download URL is assigned as
 *     a PROPERTY with spaces (`a.href = url`) so the self-containment lint --
 *     which bans the literal attribute form -- never trips. Never a static
 *     markup attribute, never a data-image URI.
 *   - Load: a hidden file input + FileReader read the text and TOLERANTLY parse
 *     it (skip + `console.warn` a malformed object, keep the rest; a structurally
 *     invalid file shows an inline message and leaves the current scene intact) --
 *     the JS counterpart to layout.py's strict raise. The optional `mosaic` /
 *     `robot.vision` blocks follow the SAME tolerance: a malformed or partial
 *     block warns and falls back to `E.defaultMosaic()` / `E.defaultVisionCone()`
 *     (never thrown -- the loud validation is layout.py's job, not the
 *     in-browser editor's), and a layout carrying neither block loads fine,
 *     leaving `E.state.mosaic` / `E.state.vision` at their current values.
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
     * A schema dict for the top-level `mosaic` block (Data Model / ADR-060),
     * matching Python `MosaicConfig.to_dict()` key-for-key. `counts` is emitted
     * as the state currently holds it (never re-derived) so a directed-mode
     * mosaic -- whose `counts` Python's own `to_dict()` still emits as `{}` --
     * round-trips byte-identical to what was authored/loaded, and a random-mode
     * mosaic keeps its exact quotas. `seed: null` means "fresh per invocation";
     * a fixed seed round-trips verbatim.
     */
    function _mosaicDict(m) {
        var counts = {};
        var srcCounts = m.counts || {};
        for (var name in srcCounts) {
            if (Object.prototype.hasOwnProperty.call(srcCounts, name)) {
                counts[name] = srcCounts[name];
            }
        }
        return {
            mode: m.mode,
            center_x_cm: m.center_x_cm,
            center_y_cm: m.center_y_cm,
            heading_deg: m.heading_deg,
            rows: m.rows,
            cols: m.cols,
            cell_w_cm: m.cell_w_cm,
            cell_h_cm: m.cell_h_cm,
            row_gap_cm: m.row_gap_cm,
            col_gap_cm: m.col_gap_cm,
            palette: m.palette.slice(),
            cells: m.cells.slice(),
            counts: counts,
            seed: (m.seed === undefined) ? null : m.seed,
        };
    }

    /**
     * A schema dict for the robot-local `robot.vision` cone (Data Model /
     * ADR-060), matching Python `VisionCone.to_dict()` key-for-key.
     * `range_cm: null` means unbounded and is ALWAYS emitted (never dropped) --
     * mirroring `VisionCone.to_dict()`'s own always-present `range_cm` key.
     */
    function _visionDict(v) {
        return {
            origin_x_cm: v.origin_x_cm,
            origin_y_cm: v.origin_y_cm,
            half_angle_deg: v.half_angle_deg,
            range_cm: (v.range_cm === undefined) ? null : v.range_cm,
        };
    }

    /**
     * A schema dict for the robot-local `robot.axle_offset` block (issue #165),
     * matching Python `AxleOffset.to_dict()` key-for-key.
     */
    function _axleOffsetDict(a) {
        return {
            forward_cm: a.forward_cm,
            lateral_cm: a.lateral_cm,
        };
    }

    /**
     * Serialise the current editor state to a schema_version-1 layout dict. Only
     * the dimension keys matching each object's shape are emitted (a rect never
     * carries radius_cm; a circle never carries w_cm/h_cm) -- mirroring
     * layout.py's `LayoutObject.to_dict`. The top-level `mosaic` block and the
     * `robot.vision` cone (T16 -- ADR-060) are always emitted here: the editor
     * state always carries a mosaic/vision (seeded by `E.defaultMosaic()` /
     * `E.defaultVisionCone()` at boot, per Contracts -- Schema keys), matching
     * how `layout.py`'s own dataclasses round-trip a present block verbatim.
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
        var robotDict = {
            grab_zone: _zoneDict(rob.grab_zone),
            box_zone: _zoneDict(rob.box_zone),
            drop_points: drops,
        };
        if (E.state.vision) {
            robotDict.vision = _visionDict(E.state.vision);
        }
        if (E.state.axleOffset) {
            robotDict.axle_offset = _axleOffsetDict(E.state.axleOffset);
        }
        var layout = {
            schema_version: 1,
            objects: objects,
            robot: robotDict,
        };
        if (E.state.mosaic) {
            layout.mosaic = _mosaicDict(E.state.mosaic);
        }
        return layout;
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

    // The edit-server heartbeat/close session (POST /ping + /close beacon,
    // ADR-057) now lives in the shared `assets/server_session.js`, inlined into
    // both the served editor and the served player so the liveness protocol has
    // a single source. It self-guards on `file:` and auto-runs on load, so there
    // is nothing to start from here.

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
        // The player's layout.py hard-requires exactly one drop point per magazine
        // lane (`len(drop_points) != MAGAZINE_LANES` -> LayoutError), and Save
        // writes whatever we keep here back verbatim. A loaded file with the wrong
        // count would round-trip as "saved cleanly" yet be un-embeddable, so fall
        // back to the default full set (and flag it) instead of emitting a layout
        // the player will reject.
        var requiredDrops = dflt.drop_points.length;
        if (drops && drops.length !== requiredDrops) {
            if (E.status) {
                E.status(
                    "drop_points needs exactly " + requiredDrops +
                    " entries (one per lane); using defaults");
            }
            drops = null;
        }
        return {
            grab_zone: grab || dflt.grab_zone,
            box_zone: box || dflt.box_zone,
            drop_points: drops || dflt.drop_points,
        };
    }

    // ----- tolerant mosaic / vision-cone ingest (T16 -- ADR-060) -----------
    // The mirror image of layout.py's `validate_mosaic_config` /
    // `validate_vision_cone` (AC3 rules), but NEVER throws: a malformed or
    // partial block warns via `console.warn` and the caller falls back to
    // `E.defaultMosaic()` / `E.defaultVisionCone()`. The loud, precise-key-path
    // raise stays layout.py's job (Error Handling); the in-browser editor must
    // survive any upload, however broken, without crashing.

    var _MOSAIC_CELL_COUNT = 12;   // rows*cols MUST equal this (Config::VISION_CELL_COUNT)
    var _MOSAIC_MODES = { random: true, directed: true };

    /**
     * Prototype-safe own-key test. Every membership check on the tolerant-ingest
     * path routes through here so a hostile upload that shadows `hasOwnProperty`
     * (e.g. a `counts`/`palette` colour literally named "hasOwnProperty") can
     * never turn a lookup into an uncaught `TypeError` -- the ingest NEVER throws.
     */
    function _hasOwn(obj, key) {
        return Object.prototype.hasOwnProperty.call(obj, key);
    }

    /**
     * The canonical colour set as an own-key map, single-sourced from
     * `MOSAIC_META.names` (the unconditional editor-emitted colour constant from
     * `mosaic.py` -- ADR-060; never a hand-duplicated list). Returns `null` when
     * the constant is absent (a legacy/served page): the caller then SKIPS the
     * canonical-membership gate and enforces only the structural rules, so a
     * page missing the vocabulary degrades gracefully rather than rejecting
     * every palette. Mirrors `editor_core.js::_paletteNames`'s tolerant read.
     */
    function _canonicalColorSet() {
        if (typeof MOSAIC_META !== "undefined" && MOSAIC_META && MOSAIC_META.names &&
            typeof MOSAIC_META.names.length === "number") {
            var set = {};
            for (var i = 0; i < MOSAIC_META.names.length; i++) {
                set[MOSAIC_META.names[i]] = true;
            }
            return set;
        }
        return null;
    }

    /** True for a finite, non-negative integer (seed / counts quota). */
    function _nonNegInt(v) {
        return typeof v === "number" && isFinite(v) && v >= 0 && Math.floor(v) === v;
    }

    /** True for a finite, positive integer (rows / cols). */
    function _posInt(v) {
        return typeof v === "number" && isFinite(v) && v > 0 && Math.floor(v) === v;
    }

    /**
     * Tolerantly parse the top-level `mosaic` block. Returns a well-formed
     * mosaic state object, or `null` (with a `console.warn`) when `data` is
     * absent or fails any AC3-equivalent check -- the caller falls back to
     * `E.defaultMosaic()`. Mirrors `mosaic.validate_mosaic_config` (Python)'s
     * rules -- structure, geometry, palette/cell/counts colour membership
     * (canonical via `MOSAIC_META.names` when present + palette-subset), the
     * directed-no-null invariant (random `counts` may sum to any non-negative
     * number, ADR-063), and the seed rule -- but tolerant (warn + default)
     * instead of loud (raise). The one place it is a
     * SUBSET, not a full mirror: on a page missing the emitted `MOSAIC_META`
     * constant the canonical-colour gate is skipped (structural rules still
     * hold), since there is no in-browser single source to check against.
     */
    function _parseMosaic(data) {
        if (data === undefined) {
            return undefined; // no key at all -- distinct from "present but bad"
        }
        if (!data || typeof data !== "object") {
            console.warn("layout: mosaic is not an object; using defaults");
            return null;
        }
        if (typeof data.mode !== "string" || !_hasOwn(_MOSAIC_MODES, data.mode)) {
            console.warn("layout: mosaic.mode must be 'random' or 'directed'; using defaults");
            return null;
        }
        if (!_posInt(data.rows) || !_posInt(data.cols) || data.rows * data.cols !== _MOSAIC_CELL_COUNT) {
            console.warn(
                "layout: mosaic.rows * mosaic.cols must equal " + _MOSAIC_CELL_COUNT +
                "; using defaults");
            return null;
        }
        if (!_finite(data.center_x_cm) || !_finite(data.center_y_cm) || !_finite(data.heading_deg)) {
            console.warn("layout: mosaic center_*/heading_deg must be finite numbers; using defaults");
            return null;
        }
        if (!_finite(data.cell_w_cm) || data.cell_w_cm <= 0 ||
            !_finite(data.cell_h_cm) || data.cell_h_cm <= 0) {
            console.warn("layout: mosaic.cell_w_cm/cell_h_cm must be finite numbers > 0; using defaults");
            return null;
        }
        if (!_finite(data.row_gap_cm) || data.row_gap_cm < 0 ||
            !_finite(data.col_gap_cm) || data.col_gap_cm < 0) {
            console.warn("layout: mosaic.row_gap_cm/col_gap_cm must be finite numbers >= 0; using defaults");
            return null;
        }
        if (!Array.isArray(data.palette) || data.palette.length === 0) {
            console.warn("layout: mosaic.palette must be a non-empty list; using defaults");
            return null;
        }
        // Each palette entry must be a known canonical colour (Python's
        // `_validate_palette` gates on `COLOR_NAMES`). `cells`/`counts` are then
        // gated on palette membership below, so a canonical palette transitively
        // rejects any non-canonical cell/count colour -- the same layering Python
        // uses. `_canonicalColorSet()` is `null` on a page missing `MOSAIC_META`;
        // there we skip only THIS gate (structural rules still apply).
        var canonical = _canonicalColorSet();
        var paletteSet = {};
        for (var p = 0; p < data.palette.length; p++) {
            var pname = data.palette[p];
            if (typeof pname !== "string" || _hasOwn(paletteSet, pname)) {
                console.warn("layout: mosaic.palette has an invalid or duplicate colour; using defaults");
                return null;
            }
            if (canonical !== null && !_hasOwn(canonical, pname)) {
                console.warn(
                    "layout: mosaic.palette[" + p + "] '" + pname +
                    "' is not a known colour; using defaults");
                return null;
            }
            paletteSet[pname] = true;
        }
        if (!Array.isArray(data.cells) || data.cells.length !== _MOSAIC_CELL_COUNT) {
            console.warn(
                "layout: mosaic.cells must have exactly " + _MOSAIC_CELL_COUNT +
                " entries; using defaults");
            return null;
        }
        var nullCount = 0;
        for (var c = 0; c < data.cells.length; c++) {
            var cell = data.cells[c];
            if (cell === null) {
                nullCount++;
                continue;
            }
            if (typeof cell !== "string" || !_hasOwn(paletteSet, cell)) {
                console.warn("layout: mosaic.cells[" + c + "] is not null or a palette colour; using defaults");
                return null;
            }
        }
        if (data.mode === "directed") {
            if (nullCount !== 0) {
                console.warn("layout: mosaic.mode='directed' requires every cell pinned; using defaults");
                return null;
            }
        } else {
            var counts = (data.counts === undefined) ? {} : data.counts;
            if (!counts || typeof counts !== "object" || Array.isArray(counts)) {
                console.warn("layout: mosaic.counts must be an object; using defaults");
                return null;
            }
            for (var name in counts) {
                // `counts` is user-supplied upload data -- a payload like
                // {"counts":{"hasOwnProperty":12}} shadows the method, so call
                // it prototype-safely (never `counts.hasOwnProperty(...)`) or a
                // TypeError escapes _ingest and crashes the editor on Load.
                if (!Object.prototype.hasOwnProperty.call(counts, name)) {
                    continue;
                }
                if (!_hasOwn(paletteSet, name) || !_nonNegInt(counts[name])) {
                    console.warn("layout: mosaic.counts has an invalid colour or quota; using defaults");
                    return null;
                }
            }
            // sum(counts) is NOT constrained to nullCount (ADR-063): an under-fill
            // leaves the leftover null cells as air, an over-fill keeps a random
            // subset -- both resolved by roll_mosaic, so any non-negative sum is ok.
        }
        if (data.seed !== undefined && data.seed !== null && !_nonNegInt(data.seed)) {
            console.warn("layout: mosaic.seed must be null or a non-negative integer; using defaults");
            return null;
        }
        return {
            mode: data.mode,
            center_x_cm: +data.center_x_cm,
            center_y_cm: +data.center_y_cm,
            heading_deg: +data.heading_deg,
            rows: data.rows,
            cols: data.cols,
            cell_w_cm: +data.cell_w_cm,
            cell_h_cm: +data.cell_h_cm,
            row_gap_cm: +data.row_gap_cm,
            col_gap_cm: +data.col_gap_cm,
            palette: data.palette.slice(),
            cells: data.cells.slice(),
            counts: (data.counts === undefined) ? {} : data.counts,
            seed: (data.seed === undefined) ? null : data.seed,
        };
    }

    /**
     * Tolerantly parse the `robot.vision` cone. Returns a well-formed vision
     * state object, or `null` (with a `console.warn`) when `data` is absent or
     * fails any AC3-equivalent check -- the caller falls back to
     * `E.defaultVisionCone()`. Mirrors `mosaic.validate_vision_cone` (Python),
     * but tolerant instead of loud.
     */
    function _parseVision(data) {
        if (data === undefined) {
            return undefined; // no key at all -- distinct from "present but bad"
        }
        if (!data || typeof data !== "object") {
            console.warn("layout: robot.vision is not an object; using defaults");
            return null;
        }
        if (!_finite(data.origin_x_cm) || !_finite(data.origin_y_cm)) {
            console.warn("layout: robot.vision.origin_x_cm/origin_y_cm must be finite numbers; using defaults");
            return null;
        }
        if (!_finite(data.half_angle_deg) || data.half_angle_deg <= 0 || data.half_angle_deg >= 90) {
            console.warn("layout: robot.vision.half_angle_deg must be strictly in (0, 90); using defaults");
            return null;
        }
        var range = data.range_cm;
        if (range !== undefined && range !== null && (!_finite(range) || range <= 0)) {
            console.warn("layout: robot.vision.range_cm must be null or a finite number > 0; using defaults");
            return null;
        }
        return {
            origin_x_cm: +data.origin_x_cm,
            origin_y_cm: +data.origin_y_cm,
            half_angle_deg: +data.half_angle_deg,
            range_cm: (range === undefined || range === null) ? null : +range,
        };
    }

    /**
     * Tolerantly parse the robot-local `robot.axle_offset` block (issue #165).
     * Returns a well-formed offset object, `null` (with a `console.warn`) when
     * `data` is present but malformed, or `undefined` when the key is absent --
     * the caller then leaves `E.state.axleOffset` untouched. Mirrors
     * `_parseVision`'s tri-state contract, but there is no "default axle
     * offset" to fall back to on a malformed block: an offset redefines the
     * robot-local origin (Data Model), so a bad upload drops to NO offset
     * (axle-anchored, today's render) rather than a synthesized non-zero one.
     */
    function _parseAxleOffset(data) {
        if (data === undefined) {
            return undefined; // no key at all -- distinct from "present but bad"
        }
        if (!data || typeof data !== "object") {
            console.warn("layout: robot.axle_offset is not an object; dropping offset");
            return null;
        }
        if (!_finite(data.forward_cm) || !_finite(data.lateral_cm)) {
            console.warn(
                "layout: robot.axle_offset.forward_cm/lateral_cm must be finite numbers; " +
                "dropping offset");
            return null;
        }
        return {
            forward_cm: +data.forward_cm,
            lateral_cm: +data.lateral_cm,
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
        // A layout with no mosaic/vision block leaves the current state alone
        // (it already carries `E.defaultMosaic()`/`E.defaultVisionCone()` from
        // boot); a PRESENT-but-malformed block warns and resets to defaults
        // rather than leaving stale/partial data in place.
        var robotSection = (data.robot && typeof data.robot === "object") ? data.robot : {};
        var mosaic = _parseMosaic(data.mosaic);
        if (mosaic !== undefined) {
            E.state.mosaic = mosaic || E.defaultMosaic();
        }
        var vision = _parseVision(robotSection.vision);
        if (vision !== undefined) {
            E.state.vision = vision || E.defaultVisionCone();
        }
        var axleOffset = _parseAxleOffset(robotSection.axle_offset);
        if (axleOffset !== undefined) {
            E.state.axleOffset = axleOffset || null;
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
})();
