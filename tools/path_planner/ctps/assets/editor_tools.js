/* CTPS — Layout Editor: tools (Rect/Circle rubber-band + select/drag/delete +
 * Properties panel + Mosaic/vision-cone property-panel editing).
 *
 * The pointer-interaction + Properties layer of the three-tab offline editor
 * (object/cargo plan, T8 -- issue #57; randomized-vision plan, T15 -- ADR-060),
 * built on editor_core.js's `window.CTPS_EDITOR` namespace + its `unproject`
 * (pixel->world) inverse:
 *
 *   - Field tab: Rect/Circle rubber-band draw (a new field object), click to
 *     select the topmost object, drag to move it, Delete/Backspace to remove it.
 *   - Robot tab: click a manipulator zone or drop point to select it, drag to
 *     move it, and (Rect tool) rubber-band to re-shape the selected zone -- all
 *     in the robot-LOCAL frame the Robot tab renders directly. The vision-cone
 *     fields (`data-mosaic-vision`) bind directly onto `E.state.vision` (T15).
 *   - Properties panel wiring: fills + edits EVERY schema field for the current
 *     selection -- name / type (Tile|Cargo|Fixed) / color / x,y for both shapes,
 *     w,h + a heading rotation input for a rect (heading hidden for a circle),
 *     radius for a circle; for a robot selection only the geometric fields show.
 *   - Mosaic tab (T15): the grid geometry/gaps/mode inputs (`data-mosaic-field`
 *     / `data-mosaic-mode`) bind onto `E.state.mosaic`; the palette
 *     (`data-mosaic-palette-list`) is populated as toggleable chips from
 *     `MOSAIC_META`; the 4x3 pin grid (`data-mosaic-grid`) cycles each cell
 *     `unset -> palette[0] -> ... -> palette[n-1] -> unset` on click; the counts
 *     inputs (`data-mosaic-counts-list`) author the random-mode quotas with a
 *     live "sum == #null cells" hint. `initMosaicPanel`/`refreshMosaicPanel` are
 *     the entrypoints T17 calls to (re)populate the palette/grid/counts from
 *     state (e.g. after a Load).
 *
 * Vanilla ES5. Never sets an inline element style: show/hide is the `hidden`
 * property only, and colour swatches ride the `--ctps-cell-color` custom
 * property (the ADR-053 carve-out) -- the canvas remains the sole freeform
 * colour surface.
 */

"use strict";

(function () {
    var E = (window.CTPS_EDITOR = window.CTPS_EDITOR || {});

    // ----- interaction constants -------------------------------------------
    var MIN_CM = 1;             // smallest rubber-band size that creates/reshapes
    var DROP_HIT_PX = 9;        // click within this of a drop point selects it
    var DEFAULT_TYPE = "tile";  // new-object type
    var DEFAULT_COLOR = "#57b36b"; // new-object fill (a tile green)
    var HEX_RE = /^#[0-9A-Fa-f]{6}$/;
    var MOSAIC_CELL_COUNT = 12; // rows*cols MUST equal this (Config::VISION_CELL_COUNT)

    // ----- small numeric helpers -------------------------------------------

    /** Round to 0.1 cm/deg so authored + dragged values stay clean. */
    function _round(v) {
        return Math.round(v * 10) / 10;
    }

    /** Parse a finite number from an input, falling back to `dflt`. */
    function _num(v, dflt) {
        var n = parseFloat(v);
        return isFinite(n) ? _round(n) : dflt;
    }

    /** Parse a finite, strictly-positive number, falling back to `dflt`. */
    function _posNum(v, dflt) {
        var n = parseFloat(v);
        return (isFinite(n) && n > 0) ? _round(n) : dflt;
    }

    /** Parse a finite, non-negative number, falling back to `dflt`. */
    function _nonNegNum(v, dflt) {
        var n = parseFloat(v);
        return (isFinite(n) && n >= 0) ? _round(n) : dflt;
    }

    /** Parse a positive integer, falling back to `dflt`. */
    function _posInt(v, dflt) {
        var n = parseInt(v, 10);
        return (isFinite(n) && n > 0) ? n : dflt;
    }

    /** Parse a non-negative integer, falling back to `dflt`. */
    function _nonNegInt(v, dflt) {
        var n = parseInt(v, 10);
        return (isFinite(n) && n >= 0) ? n : dflt;
    }

    /** Format a number for a numeric input (rounded, no trailing zeros). */
    function _fmt(v) {
        return String(_round(v));
    }

    // ----- status line ------------------------------------------------------

    /** Write a one-line message into the stage status readout. */
    function status(msg) {
        if (E.dom && E.dom.status) {
            E.dom.status.textContent = msg;
        }
    }

    /** Show the live world coordinates under the pointer. */
    function _statusCoord(w) {
        status("x " + _round(w.x) + "   y " + _round(w.y));
    }

    // ----- pointer geometry -------------------------------------------------

    /** Convert a pointer/mouse event to canvas-relative CSS px. */
    function _canvasXY(ev) {
        var rect = E.canvas.getBoundingClientRect();
        return { px: ev.clientX - rect.left, py: ev.clientY - rect.top };
    }

    /** Capture the pointer so a drag continues off-canvas (best-effort). */
    function _capture(ev) {
        try {
            if (E.canvas.setPointerCapture && ev.pointerId != null) {
                E.canvas.setPointerCapture(ev.pointerId);
            }
        } catch (e) { /* capture is a nicety; ignore failures */ }
    }

    /** Release a previously captured pointer (best-effort). */
    function _release(ev) {
        try {
            if (E.canvas.releasePointerCapture && ev.pointerId != null) {
                E.canvas.releasePointerCapture(ev.pointerId);
            }
        } catch (e) { /* ignore */ }
    }

    // ----- hit-testing ------------------------------------------------------

    /** True when world point (wx,wy) lies inside object `o` (rotated for a rect). */
    function _pointInObject(o, wx, wy) {
        var dx = wx - o.x_cm;
        var dy = wy - o.y_cm;
        if (o.shape === "circle") {
            return Math.hypot(dx, dy) <= (o.radius_cm || 0);
        }
        var h = (o.heading_deg || 0) * Math.PI / 180;
        // Inverse-rotate the delta into the object's local frame.
        var lx = dx * Math.cos(h) + dy * Math.sin(h);
        var ly = -dx * Math.sin(h) + dy * Math.cos(h);
        return Math.abs(lx) <= (o.w_cm || 0) / 2 && Math.abs(ly) <= (o.h_cm || 0) / 2;
    }

    /** The topmost object under a world point (last drawn wins), or null. */
    function _hitObject(wx, wy) {
        var objs = E.state.objects;
        for (var i = objs.length - 1; i >= 0; i--) {
            if (_pointInObject(objs[i], wx, wy)) {
                return { kind: "object", id: objs[i].id };
            }
        }
        return null;
    }

    /** True when local point (wx,wy) is inside axis-aligned local rect `z`. */
    function _inZone(z, wx, wy) {
        if (!z) {
            return false;
        }
        return Math.abs(wx - z.x_cm) <= z.w_cm / 2 && Math.abs(wy - z.y_cm) <= z.h_cm / 2;
    }

    /** The robot zone/drop point under a local point (drops first), or null. */
    function _hitRobot(view, wx, wy) {
        var rob = E.state.robot;
        if (!rob) {
            return null;
        }
        var thresh = DROP_HIT_PX / view.scale;
        var drops = rob.drop_points || [];
        for (var i = 0; i < drops.length; i++) {
            var d = drops[i];
            if (d && Math.hypot(wx - d.x_cm, wy - d.y_cm) <= thresh) {
                return { kind: "drop", index: i };
            }
        }
        if (_inZone(rob.grab_zone, wx, wy)) {
            return { kind: "grab_zone" };
        }
        if (_inZone(rob.box_zone, wx, wy)) {
            return { kind: "box_zone" };
        }
        return null;
    }

    /** The world-cm centre of a selection (for a drag grab offset). */
    function _selectionCenter(hit) {
        if (hit.kind === "object") {
            var o = E.objectById(hit.id);
            return o ? { x: o.x_cm, y: o.y_cm } : { x: 0, y: 0 };
        }
        if (hit.kind === "drop") {
            var dp = E.state.robot.drop_points[hit.index];
            return { x: dp.x_cm, y: dp.y_cm };
        }
        var z = E.state.robot[hit.kind];
        return { x: z.x_cm, y: z.y_cm };
    }

    // ----- object creation / mutation --------------------------------------

    /** A friendly default name for a newly-created object. */
    function _defaultName() {
        return "object " + (E.state.objects.length + 1);
    }

    /** Create a field object of `shape` from committed draft geometry, select it. */
    function _createObject(shape, g) {
        var o = {
            id: E.nextId(),
            name: _defaultName(),
            shape: shape,
            type: DEFAULT_TYPE,
            color: DEFAULT_COLOR,
            x_cm: _round(g.x),
            y_cm: _round(g.y),
            heading_deg: 0,
        };
        if (shape === "rect") {
            o.w_cm = _round(g.w);
            o.h_cm = _round(g.h);
        } else {
            o.radius_cm = _round(g.radius);
        }
        E.state.objects.push(o);
        E.state.selection = { kind: "object", id: o.id };
        refreshProps();
        status("added " + o.name);
    }

    /** Remove the object `id`, clearing the selection + Properties panel. */
    function _deleteObject(id) {
        var objs = E.state.objects;
        for (var i = 0; i < objs.length; i++) {
            if (objs[i].id === id) {
                objs.splice(i, 1);
                break;
            }
        }
        E.state.selection = null;
        refreshProps();
        E.redraw();
        if (E.refreshSaveState) {
            E.refreshSaveState();
        }
    }

    /** Delete the currently-selected object (no-op for robot zones/points). */
    function deleteSelected() {
        var s = E.state.selection;
        if (s && s.kind === "object") {
            _deleteObject(s.id);
        }
    }

    /** Move the current drag target so its centre tracks world point `w`. */
    function _applyDrag(w) {
        var d = E.state.drag;
        var s = d.selection;
        var nx = w.x - d.offX;
        var ny = w.y - d.offY;
        if (s.kind === "object") {
            var o = E.objectById(s.id);
            if (o) {
                o.x_cm = _round(nx);
                o.y_cm = _round(ny);
            }
        } else if (s.kind === "drop") {
            var dp = E.state.robot.drop_points[s.index];
            dp.x_cm = _round(nx);
            dp.y_cm = _round(ny);
        } else {
            var z = E.state.robot[s.kind];
            z.x_cm = _round(nx);
            z.y_cm = _round(ny);
        }
    }

    /** Commit a finished rubber-band draft into a new object / reshaped zone. */
    function _commitDraft() {
        var d = E.state.draft;
        var dx = d.x1 - d.x0;
        var dy = d.y1 - d.y0;
        if (E.state.activeTab === "robot") {
            if (d.tool !== "rect") {
                return; // circles are not authored on the Robot tab
            }
            var w = Math.abs(dx);
            var h = Math.abs(dy);
            var sel = E.state.selection;
            var kind = (sel && (sel.kind === "grab_zone" || sel.kind === "box_zone"))
                ? sel.kind : "grab_zone";
            if (w < MIN_CM || h < MIN_CM) {
                status("drag a larger box to set the " + kind);
                return;
            }
            var z = E.state.robot[kind];
            z.x_cm = _round((d.x0 + d.x1) / 2);
            z.y_cm = _round((d.y0 + d.y1) / 2);
            z.w_cm = _round(w);
            z.h_cm = _round(h);
            E.state.selection = { kind: kind };
            refreshProps();
            return;
        }
        if (d.tool === "circle") {
            var radius = Math.hypot(dx, dy);
            if (radius < MIN_CM) {
                return;
            }
            _createObject("circle", { x: d.x0, y: d.y0, radius: radius });
        } else {
            var ww = Math.abs(dx);
            var hh = Math.abs(dy);
            if (ww < MIN_CM || hh < MIN_CM) {
                return;
            }
            _createObject("rect", { x: (d.x0 + d.x1) / 2, y: (d.y0 + d.y1) / 2, w: ww, h: hh });
        }
    }

    // ----- pointer event handlers ------------------------------------------

    /** Begin a draw (rect/circle) or a select+drag on pointer-down. */
    function onPointerDown(ev) {
        var view = E.activeView();
        if (!view) {
            return;
        }
        var pt = _canvasXY(ev);
        var w = E.unproject(view, pt.px, pt.py);
        var tab = E.state.activeTab;
        var tool = E.state.tool;

        if (tab === "mosaic") {
            return; // the Mosaic tab has no rubber-band / drag interactions
        }

        if (tool === "rect" || tool === "circle") {
            if (tab === "robot" && tool === "circle") {
                return; // no circle authoring on the Robot tab
            }
            E.state.draft = { tool: tool, x0: w.x, y0: w.y, x1: w.x, y1: w.y };
            _capture(ev);
            return;
        }

        // Select tool: pick a target, then begin a move drag on a hit.
        var hit = (tab === "robot") ? _hitRobot(view, w.x, w.y) : _hitObject(w.x, w.y);
        E.state.selection = hit;
        if (hit) {
            var center = _selectionCenter(hit);
            E.state.drag = { selection: hit, offX: w.x - center.x, offY: w.y - center.y };
        } else {
            E.state.drag = null;
        }
        refreshProps();
        E.redraw();
        _capture(ev);
    }

    /** Extend a draft, apply a drag, or update the coordinate readout. */
    function onPointerMove(ev) {
        var view = E.activeView();
        if (!view) {
            return;
        }
        var pt = _canvasXY(ev);
        var w = E.unproject(view, pt.px, pt.py);
        if (E.state.draft) {
            E.state.draft.x1 = w.x;
            E.state.draft.y1 = w.y;
            E.redraw();
            return;
        }
        if (E.state.drag) {
            _applyDrag(w);
            E.redraw();
            refreshProps();
            return;
        }
        if (ev.target === E.canvas) {
            _statusCoord(w);
        }
    }

    /** Commit a draft / end a drag on pointer-up. */
    function onPointerUp(ev) {
        var changed = false;
        if (E.state.draft) {
            _commitDraft();
            E.state.draft = null;
            E.redraw();
            changed = true;
        }
        if (E.state.drag) {
            E.state.drag = null;
            changed = true;
        }
        _release(ev);
        if (changed && E.refreshSaveState) {
            E.refreshSaveState();
        }
    }

    /** Delete/Backspace removes the selected object (unless a field is focused). */
    function onKeyDown(ev) {
        if (ev.key !== "Delete" && ev.key !== "Backspace") {
            return;
        }
        var s = E.state.selection;
        if (!s || s.kind !== "object") {
            return;
        }
        if (_isEditing(ev.target)) {
            return; // let Backspace edit an input instead of deleting the object
        }
        _deleteObject(s.id);
        ev.preventDefault();
    }

    /** True when the event target is a form control being edited. */
    function _isEditing(el) {
        if (!el) {
            return false;
        }
        var tag = (el.tagName || "").toLowerCase();
        return tag === "input" || tag === "select" || tag === "textarea";
    }

    // ----- Properties panel -------------------------------------------------

    /** Toggle an element's visibility via the `hidden` property. */
    function _setVisible(el, visible) {
        if (el) {
            el.hidden = !visible;
        }
    }

    /** Show either the "no object selected" hint or the field set. */
    function _showEmpty(isEmpty) {
        var dom = E.dom;
        _setVisible(dom.propsEmpty, isEmpty);
        _setVisible(dom.propsFields, !isEmpty);
    }

    /** Populate the panel for a selected field object (every schema field). */
    function _fillObjectProps(o) {
        if (!o) {
            _showEmpty(true);
            return;
        }
        var dom = E.dom;
        _showEmpty(false);
        _setVisible(dom.nameField, true);
        _setVisible(dom.typeField, true);
        _setVisible(dom.colorField, true);
        _setVisible(dom.del, true);
        dom.name.value = o.name;
        dom.type.value = o.type;
        dom.color.value = HEX_RE.test(o.color) ? o.color : "#000000";
        dom.x.value = _fmt(o.x_cm);
        dom.y.value = _fmt(o.y_cm);
        if (o.shape === "rect") {
            _setVisible(dom.rectFields, true);
            _setVisible(dom.circleFields, false);
            _setVisible(dom.headingField, true);
            dom.w.value = _fmt(o.w_cm);
            dom.h.value = _fmt(o.h_cm);
            dom.heading.value = _fmt(o.heading_deg);
        } else {
            _setVisible(dom.rectFields, false);
            _setVisible(dom.circleFields, true);
            dom.radius.value = _fmt(o.radius_cm);
        }
    }

    /** Populate the panel for a selected robot zone / drop point (geometry only). */
    function _fillRobotProps(s) {
        var dom = E.dom;
        _showEmpty(false);
        _setVisible(dom.nameField, false);
        _setVisible(dom.typeField, false);
        _setVisible(dom.colorField, false);
        _setVisible(dom.del, false);
        _setVisible(dom.headingField, false);
        _setVisible(dom.circleFields, false);
        if (s.kind === "drop") {
            var dp = E.state.robot.drop_points[s.index];
            _setVisible(dom.rectFields, false);
            dom.x.value = _fmt(dp.x_cm);
            dom.y.value = _fmt(dp.y_cm);
        } else {
            var z = E.state.robot[s.kind];
            _setVisible(dom.rectFields, true);
            dom.x.value = _fmt(z.x_cm);
            dom.y.value = _fmt(z.y_cm);
            dom.w.value = _fmt(z.w_cm);
            dom.h.value = _fmt(z.h_cm);
        }
    }

    /** Rebuild the Properties panel from the current selection. */
    function refreshProps() {
        if (!E.dom) {
            return;
        }
        var s = E.state.selection;
        if (!s) {
            _showEmpty(true);
            return;
        }
        if (s.kind === "object") {
            _fillObjectProps(E.objectById(s.id));
        } else {
            _fillRobotProps(s);
        }
        if (E.syncSwatchActive) {
            E.syncSwatchActive(); // re-light the preset matching the new selection
        }
    }

    /** Apply a Properties-panel edit to the current selection, then repaint. */
    function onPropChange() {
        var s = E.state.selection;
        if (!s) {
            return;
        }
        var dom = E.dom;
        if (s.kind === "object") {
            var o = E.objectById(s.id);
            if (!o) {
                return;
            }
            o.name = dom.name.value;
            o.type = dom.type.value;
            if (HEX_RE.test(dom.color.value)) {
                o.color = dom.color.value;
            }
            o.x_cm = _num(dom.x.value, o.x_cm);
            o.y_cm = _num(dom.y.value, o.y_cm);
            if (o.shape === "rect") {
                o.w_cm = _posNum(dom.w.value, o.w_cm);
                o.h_cm = _posNum(dom.h.value, o.h_cm);
                o.heading_deg = _num(dom.heading.value, o.heading_deg);
            } else {
                o.radius_cm = _posNum(dom.radius.value, o.radius_cm);
            }
        } else if (s.kind === "drop") {
            var dp = E.state.robot.drop_points[s.index];
            dp.x_cm = _num(dom.x.value, dp.x_cm);
            dp.y_cm = _num(dom.y.value, dp.y_cm);
        } else {
            var z = E.state.robot[s.kind];
            z.x_cm = _num(dom.x.value, z.x_cm);
            z.y_cm = _num(dom.y.value, z.y_cm);
            z.w_cm = _posNum(dom.w.value, z.w_cm);
            z.h_cm = _posNum(dom.h.value, z.h_cm);
        }
        E.redraw();
        if (E.refreshSaveState) {
            E.refreshSaveState();
        }
        if (E.syncSwatchActive) {
            E.syncSwatchActive(); // native-picker edits re-light a matching preset
        }
    }

    // ----- Mosaic tab: geometry / mode fields (data-mosaic-field / -mode) ---
    //
    // `E.state.mosaic` (seeded by editor_core.js's `defaultMosaic()`) is mutated
    // directly, matching the convention `onPropChange` already uses for
    // `E.state.robot` -- no intermediate copy, no dedicated "commit" step: every
    // input listener writes straight through and calls `E.redraw()`.

    /** The geometry keys that are INTEGER grid dimensions (parsed as ints). */
    var _MOSAIC_INT_KEYS = { rows: true, cols: true };
    /** The geometry keys that must stay non-negative (gaps; 0 is a valid gap). */
    var _MOSAIC_NONNEG_KEYS = { row_gap_cm: true, col_gap_cm: true };
    /** The geometry keys that must stay strictly positive (cell size, grid dims). */
    var _MOSAIC_POS_KEYS = { cell_w_cm: true, cell_h_cm: true, rows: true, cols: true };

    /** Parse one `data-mosaic-field` input per its key's numeric contract. */
    function _parseMosaicField(key, raw, dflt) {
        if (_MOSAIC_INT_KEYS[key]) {
            return _MOSAIC_POS_KEYS[key] ? _posInt(raw, dflt) : _nonNegInt(raw, dflt);
        }
        if (_MOSAIC_POS_KEYS[key]) {
            return _posNum(raw, dflt);
        }
        if (_MOSAIC_NONNEG_KEYS[key]) {
            return _nonNegNum(raw, dflt);
        }
        return _num(raw, dflt); // center_x_cm / center_y_cm / heading_deg: any finite
    }

    /** One `data-mosaic-field` input changed: write `E.state.mosaic[key]`, redraw. */
    function onMosaicFieldChange(ev) {
        var el = ev.target;
        var key = el.getAttribute("data-mosaic-field");
        if (!key || !E.state.mosaic) {
            return;
        }
        var mosaic = E.state.mosaic;
        mosaic[key] = _parseMosaicField(key, el.value, mosaic[key]);
        // rows/cols/cell size/gap changes reshape the grid -> the Mosaic-tab
        // bounds (and the pin-grid template) must be re-derived, so a full
        // refresh (not just a redraw) keeps the grid + preview in lockstep.
        if (key === "rows" || key === "cols") {
            refreshMosaicPanel();
        }
        E.resize(); // re-fit the Mosaic-tab bounds (geometry may have moved/grown)
        _updateCountsTotal();
        if (E.refreshSaveState) {
            E.refreshSaveState();
        }
    }

    /** Bind every `[data-mosaic-field]` input's input/change to the state write. */
    function _bindMosaicFields() {
        var els = document.querySelectorAll("[data-mosaic-field]");
        for (var i = 0; i < els.length; i++) {
            els[i].addEventListener("input", onMosaicFieldChange);
            els[i].addEventListener("change", onMosaicFieldChange);
        }
    }

    /** Reflect `E.state.mosaic`'s geometry onto the `[data-mosaic-field]` inputs. */
    function _reflectMosaicFields() {
        var mosaic = E.state.mosaic;
        if (!mosaic) {
            return;
        }
        var els = document.querySelectorAll("[data-mosaic-field]");
        for (var i = 0; i < els.length; i++) {
            var el = els[i];
            var key = el.getAttribute("data-mosaic-field");
            if (key && mosaic[key] !== undefined && mosaic[key] !== null) {
                el.value = _MOSAIC_INT_KEYS[key] ? String(mosaic[key]) : _fmt(mosaic[key]);
            }
        }
    }

    /** Set `E.state.mosaic.mode`, reflect the mode buttons + counts visibility. */
    function setMosaicMode(mode) {
        if (!E.state.mosaic || (mode !== "random" && mode !== "directed")) {
            return;
        }
        E.state.mosaic.mode = mode;
        _reflectMosaicMode();
        _refreshCountsVisibility();
        _updateCountsTotal();
        E.redraw();
        if (E.refreshSaveState) {
            E.refreshSaveState();
        }
    }

    /** Reflect `E.state.mosaic.mode` onto the mode toggle buttons' active class. */
    function _reflectMosaicMode() {
        var mosaic = E.state.mosaic;
        if (!mosaic) {
            return;
        }
        var els = document.querySelectorAll("[data-mosaic-mode]");
        for (var i = 0; i < els.length; i++) {
            var el = els[i];
            var isActive = el.getAttribute("data-mosaic-mode") === mosaic.mode;
            el.classList.toggle("ctps-mosaic-mode-btn--active", isActive);
        }
    }

    /** Bind the mode toggle buttons (`data-mosaic-mode="random"|"directed"`). */
    function _bindMosaicMode() {
        var els = document.querySelectorAll("[data-mosaic-mode]");
        for (var i = 0; i < els.length; i++) {
            (function (el) {
                var mode = el.getAttribute("data-mosaic-mode");
                el.addEventListener("click", function () { setMosaicMode(mode); });
            })(els[i]);
        }
    }

    // ----- Mosaic tab: palette (data-mosaic-palette-list) -------------------

    /** True when `name` is currently in `E.state.mosaic.palette`. */
    function _isPaletteActive(name) {
        var palette = E.state.mosaic && E.state.mosaic.palette;
        return !!(palette && palette.indexOf(name) !== -1);
    }

    /**
     * Toggle `name` in/out of `E.state.mosaic.palette`. Removing a colour also
     * un-pins it from every cell + count (a palette member that quietly stayed
     * pinned would violate the AC3 "pin must be in palette" rule the moment it
     * is saved) and re-derives the counts to keep the "sum == #null" hint live.
     */
    function _toggleMosaicPaletteColor(name) {
        var mosaic = E.state.mosaic;
        if (!mosaic) {
            return;
        }
        var palette = mosaic.palette || (mosaic.palette = []);
        var idx = palette.indexOf(name);
        if (idx === -1) {
            palette.push(name);
        } else {
            palette.splice(idx, 1);
            // Un-pin any cell that pinned the removed colour.
            var cells = mosaic.cells || [];
            for (var i = 0; i < cells.length; i++) {
                if (cells[i] === name) {
                    cells[i] = null;
                }
            }
            // Drop the removed colour's quota (no longer authorable).
            if (mosaic.counts) {
                delete mosaic.counts[name];
            }
        }
        refreshMosaicPanel();
        E.redraw();
        if (E.refreshSaveState) {
            E.refreshSaveState();
        }
    }

    /**
     * (Re)build the palette chip list (`data-mosaic-palette-list`) from
     * `MOSAIC_META.names`/`.hex`: one toggleable chip per known colour, a swatch
     * + its name, `--active` when the colour is currently in
     * `E.state.mosaic.palette`. Each chip's swatch carries `--ctps-cell-color`
     * (the SAME custom property the pin-grid cells read) so a palette chip and
     * every pinned cell in that colour always agree visually.
     */
    function _renderMosaicPalette() {
        var host = document.querySelector("[data-mosaic-palette-list]");
        if (!host) {
            return;
        }
        while (host.firstChild) {
            host.removeChild(host.firstChild);
        }
        var meta = (typeof MOSAIC_META !== "undefined") ? MOSAIC_META : null;
        var names = (meta && meta.names) ? meta.names : [];
        var hex = (meta && meta.hex) ? meta.hex : {};
        for (var i = 0; i < names.length; i++) {
            (function (name) {
                var chip = document.createElement("button");
                chip.type = "button";
                chip.className = "ctps-mosaic-palette-chip";
                chip.classList.toggle("ctps-mosaic-palette-chip--active",
                    _isPaletteActive(name));
                var swatch = document.createElement("span");
                swatch.className = "ctps-mosaic-palette-swatch";
                if (hex[name]) {
                    swatch.style.setProperty("--ctps-cell-color", hex[name]);
                }
                var label = document.createElement("span");
                label.textContent = name;
                chip.appendChild(swatch);
                chip.appendChild(label);
                chip.addEventListener("click", function () {
                    _toggleMosaicPaletteColor(name);
                });
                host.appendChild(chip);
            })(names[i]);
        }
    }

    // ----- Mosaic tab: 4x3 pin grid (data-mosaic-grid) -----------------------

    /**
     * Cycle cell `i`'s pin per the Contracts "Pin-grid interaction":
     * `unset(rolled) -> palette[0] -> palette[1] -> ... -> palette[n-1] -> unset`.
     * An empty palette leaves the cell unset (nothing to cycle to).
     */
    function _cycleMosaicCell(i) {
        var mosaic = E.state.mosaic;
        if (!mosaic || !mosaic.cells) {
            return;
        }
        var palette = mosaic.palette || [];
        if (palette.length === 0) {
            mosaic.cells[i] = null;
            return;
        }
        var current = mosaic.cells[i];
        var nextIdx = (current === null) ? 0 : palette.indexOf(current) + 1;
        mosaic.cells[i] = (nextIdx >= palette.length) ? null : palette[nextIdx];
    }

    /** Click handler for one pin-grid cell: cycle it, refresh the grid, redraw. */
    function onMosaicCellClick(i) {
        _cycleMosaicCell(i);
        _renderMosaicGrid(); // repaint just the grid (counts total may change)
        _updateCountsTotal();
        E.redraw();
        if (E.refreshSaveState) {
            E.refreshSaveState();
        }
    }

    /**
     * (Re)build the 4x3 (or `rows`x`cols`) pin grid (`data-mosaic-grid`) as
     * `rows*cols` `.ctps-mosaic-cell` cells in row-major order (`i =
     * row*cols+col`, row 0 top-left, matching the Contracts cell-ordering
     * convention). Sets the container's `grid-template-columns` to the
     * authored `cols` via `style.setProperty` (never a direct `.style.<prop> =`
     * assignment) -- a LAYOUT dimension driven by the authored grid shape, not
     * a colour/appearance value, so it is not an ADR-053 violation (the CSS
     * custom-property carve-out already covers colour; the cell COUNT/shape is
     * structural state the static stylesheet cannot know in advance).
     *
     * Each cell renders EITHER `.ctps-mosaic-cell--unset` (a dashed outline,
     * unset/rolled) OR `.ctps-mosaic-cell--filled` (a solid swatch) with
     * `--ctps-cell-color` set from `MOSAIC_META.hex[color]` -- the T12 CSS
     * reads that custom property, so a filled WHITE cell is a bordered swatch
     * clearly distinct from the dashed-outline unset state.
     */
    function _renderMosaicGrid() {
        var host = document.querySelector("[data-mosaic-grid]");
        if (!host) {
            return;
        }
        var mosaic = E.state.mosaic;
        while (host.firstChild) {
            host.removeChild(host.firstChild);
        }
        if (!mosaic) {
            return;
        }
        var cols = mosaic.cols || 3;
        var rows = mosaic.rows || 4;
        // `setProperty` (never a direct `.style.<prop> =` assignment, per the
        // project's no-inline-style-assignment convention) -- `grid-template-
        // columns` is a STRUCTURAL layout dimension driven by the authored grid
        // shape (rows/cols), not a colour/appearance value, but the same DOM API
        // channel applies.
        host.style.setProperty(
            "grid-template-columns", "repeat(" + cols + ", var(--ctps-mosaic-cell-size))");
        var meta = (typeof MOSAIC_META !== "undefined") ? MOSAIC_META : null;
        var hex = (meta && meta.hex) ? meta.hex : {};
        var cells = mosaic.cells || [];
        var total = rows * cols;
        for (var i = 0; i < total; i++) {
            (function (index) {
                var color = cells[index] !== undefined ? cells[index] : null;
                var cell = document.createElement("button");
                cell.type = "button";
                cell.className = "ctps-mosaic-cell";
                cell.setAttribute("data-mosaic-cell-index", String(index));
                cell.setAttribute("aria-label", "Mosaic cell " + index);
                if (color) {
                    cell.classList.add("ctps-mosaic-cell--filled");
                    if (hex[color]) {
                        cell.style.setProperty("--ctps-cell-color", hex[color]);
                    }
                    cell.textContent = color.charAt(0);
                } else {
                    cell.classList.add("ctps-mosaic-cell--unset");
                    cell.textContent = "";
                }
                cell.addEventListener("click", function () {
                    onMosaicCellClick(index);
                });
                host.appendChild(cell);
            })(i);
        }
    }

    // ----- Mosaic tab: counts (data-mosaic-counts-list) ----------------------

    /** The number of `null` (rolled) cells in the current mosaic. */
    function _nullCellCount() {
        var mosaic = E.state.mosaic;
        if (!mosaic || !mosaic.cells) {
            return 0;
        }
        var n = 0;
        for (var i = 0; i < mosaic.cells.length; i++) {
            if (mosaic.cells[i] === null) {
                n++;
            }
        }
        return n;
    }

    /** The current sum of `E.state.mosaic.counts` values. */
    function _countsSum() {
        var counts = E.state.mosaic && E.state.mosaic.counts;
        if (!counts) {
            return 0;
        }
        var sum = 0;
        for (var key in counts) {
            if (Object.prototype.hasOwnProperty.call(counts, key)) {
                sum += counts[key] || 0;
            }
        }
        return sum;
    }

    /** One `data-mosaic-count` input changed: write `counts[color]`, refresh the hint. */
    function onMosaicCountChange(ev) {
        var el = ev.target;
        var color = el.getAttribute("data-mosaic-count");
        if (!color || !E.state.mosaic) {
            return;
        }
        var mosaic = E.state.mosaic;
        if (!mosaic.counts) {
            mosaic.counts = {};
        }
        mosaic.counts[color] = _nonNegInt(el.value, mosaic.counts[color] || 0);
        _updateCountsTotal();
        E.redraw();
        if (E.refreshSaveState) {
            E.refreshSaveState();
        }
    }

    /**
     * (Re)build the counts rows (`data-mosaic-counts-list`): one numeric input
     * per PALETTE colour (random mode authors these; a colour dropped from the
     * palette has no row). A UI affordance only -- the live "sum == #null
     * cells" hint is authoring guidance; the Python `validate_mosaic_config` is
     * the loud enforcement gate (Contracts).
     */
    function _renderMosaicCounts() {
        var host = document.querySelector("[data-mosaic-counts-list]");
        if (!host) {
            return;
        }
        while (host.firstChild) {
            host.removeChild(host.firstChild);
        }
        var mosaic = E.state.mosaic;
        if (!mosaic) {
            return;
        }
        var meta = (typeof MOSAIC_META !== "undefined") ? MOSAIC_META : null;
        var hex = (meta && meta.hex) ? meta.hex : {};
        var palette = mosaic.palette || [];
        var counts = mosaic.counts || (mosaic.counts = {});
        for (var i = 0; i < palette.length; i++) {
            (function (color) {
                var row = document.createElement("div");
                row.className = "ctps-mosaic-count-row";
                var swatch = document.createElement("span");
                swatch.className = "ctps-mosaic-palette-swatch";
                if (hex[color]) {
                    swatch.style.setProperty("--ctps-cell-color", hex[color]);
                }
                var label = document.createElement("span");
                label.className = "ctps-mosaic-count-label";
                label.textContent = color;
                var input = document.createElement("input");
                input.type = "number";
                input.step = "1";
                input.min = "0";
                input.className = "ctps-mosaic-count-input";
                input.setAttribute("data-mosaic-count", color);
                input.value = String(counts[color] || 0);
                input.addEventListener("input", onMosaicCountChange);
                input.addEventListener("change", onMosaicCountChange);
                row.appendChild(swatch);
                row.appendChild(label);
                row.appendChild(input);
                host.appendChild(row);
            })(palette[i]);
        }
        _refreshCountsVisibility();
    }

    /**
     * Update the `#ctps-mosaic-count-total` live readout: `"sum(counts) / N
     * null cells"`, flagged with the `--error` modifier (T12's live-authoring
     * tint) when they disagree in random mode. Directed mode shows the pinned
     * count instead (counts are ignored/hidden there) so the readout never
     * falsely reds a fully-directed mosaic.
     */
    function _updateCountsTotal() {
        var el = document.getElementById("ctps-mosaic-count-total");
        if (!el) {
            return;
        }
        var mosaic = E.state.mosaic;
        if (!mosaic) {
            el.textContent = "";
            return;
        }
        if (mosaic.mode === "directed") {
            el.textContent = MOSAIC_CELL_COUNT + "/" + MOSAIC_CELL_COUNT + " cells pinned";
            el.classList.remove("ctps-mosaic-count-total--error");
            return;
        }
        var sum = _countsSum();
        var nulls = _nullCellCount();
        el.textContent = sum + " / " + nulls + " null cells";
        el.classList.toggle("ctps-mosaic-count-total--error", sum !== nulls);
    }

    /**
     * Directed vs random UI affordance (Contracts point 6): in `directed` mode
     * the counts UI is hidden (every cell is expected pinned directly on the
     * grid); in `random` mode it is shown. A UI-only toggle -- Python
     * `validate_mosaic_config` is the actual gate, this never blocks a save.
     */
    function _refreshCountsVisibility() {
        var mosaic = E.state.mosaic;
        var host = document.getElementById("ctps-mosaic-counts");
        var isRandom = !mosaic || mosaic.mode !== "directed";
        _setVisible(host, isRandom);
    }

    // ----- Mosaic tab: public init/refresh entrypoints (for T17) ------------

    /**
     * One-time binding of every Mosaic-tab + vision-cone control's event
     * listeners (geometry/mode/palette-click delegation is per-render since the
     * palette/grid/counts are rebuilt on every refresh; the field/mode/cone
     * inputs are static DOM so their listeners are bound once here). Safe to
     * call once at boot (T17); a missing control degrades to a no-op (querying
     * `[data-mosaic-field]` etc. simply returns an empty list on a legacy page).
     */
    function initMosaicPanel() {
        _bindMosaicFields();
        _bindMosaicMode();
        _bindVisionFields();
        refreshMosaicPanel();
    }

    /**
     * (Re)populate EVERY Mosaic-tab + vision-cone piece from the current
     * `E.state.mosaic` / `E.state.vision` -- the entrypoint T17 calls after a
     * Load (or on boot) so the palette chips / pin grid / counts rows / cone
     * fields reflect freshly-ingested state rather than stale DOM from a
     * previous layout. Idempotent -- safe to call repeatedly.
     */
    function refreshMosaicPanel() {
        _reflectMosaicFields();
        _reflectMosaicMode();
        _renderMosaicPalette();
        _renderMosaicGrid();
        _renderMosaicCounts();
        _updateCountsTotal();
        _reflectVisionFields();
    }

    // ----- Robot tab: vision-cone fields (data-mosaic-vision) ----------------

    /** Parse one `data-mosaic-vision` cone input per its key's numeric contract. */
    function _parseVisionField(key, raw, dflt) {
        if (key === "half_angle_deg") {
            return _posNum(raw, dflt);
        }
        if (key === "range_cm") {
            return _posNum(raw, dflt);
        }
        return _num(raw, dflt); // origin_x_cm / origin_y_cm: any finite
    }

    /** One `data-mosaic-vision` cone input changed: write `E.state.vision[key]`. */
    function onVisionFieldChange(ev) {
        var el = ev.target;
        var key = el.getAttribute("data-mosaic-vision");
        if (!key || !E.state.vision) {
            return;
        }
        var vision = E.state.vision;
        if (key === "range_cm") {
            // A manual edit to the numeric range field implies bounded range;
            // uncheck "Unbounded" so the two controls never disagree.
            vision.range_cm = _posNum(el.value, vision.range_cm || 1);
            var unbounded = document.getElementById("ctps-mosaic-cone-range-unbounded");
            if (unbounded) {
                unbounded.checked = false;
            }
        } else {
            vision[key] = _parseVisionField(key, el.value, vision[key]);
        }
        E.resize(); // the Robot-tab bounds depend on the cone reach
        if (E.refreshSaveState) {
            E.refreshSaveState();
        }
    }

    /**
     * The `#ctps-mosaic-cone-range-unbounded` checkbox toggled: `range_cm`
     * becomes `null` (unbounded, checked) or a numeric value (unchecked -- the
     * numeric field's current value, defaulting to a sane finite range so
     * unchecking never leaves `range_cm` at `null`).
     */
    function onVisionRangeUnboundedChange(ev) {
        var vision = E.state.vision;
        if (!vision) {
            return;
        }
        var checked = ev.target.checked;
        var rangeInput = document.getElementById("ctps-mosaic-cone-range");
        if (checked) {
            vision.range_cm = null;
            if (rangeInput) {
                rangeInput.disabled = true;
            }
        } else {
            if (rangeInput) {
                rangeInput.disabled = false;
            }
            vision.range_cm = _posNum(rangeInput ? rangeInput.value : null, 100);
            if (rangeInput) {
                rangeInput.value = _fmt(vision.range_cm);
            }
        }
        E.resize(); // unbounded vs bounded changes the Robot-tab preview reach
        if (E.refreshSaveState) {
            E.refreshSaveState();
        }
    }

    /** Bind the vision-cone fields + the unbounded-range checkbox. */
    function _bindVisionFields() {
        var els = document.querySelectorAll("[data-mosaic-vision]");
        for (var i = 0; i < els.length; i++) {
            els[i].addEventListener("input", onVisionFieldChange);
            els[i].addEventListener("change", onVisionFieldChange);
        }
        var unbounded = document.querySelector("[data-mosaic-vision-unbounded]");
        if (unbounded) {
            unbounded.addEventListener("change", onVisionRangeUnboundedChange);
        }
    }

    /** Reflect `E.state.vision` onto the `[data-mosaic-vision]` inputs + the checkbox. */
    function _reflectVisionFields() {
        var vision = E.state.vision;
        if (!vision) {
            return;
        }
        var isUnbounded = (vision.range_cm === null || vision.range_cm === undefined);
        var els = document.querySelectorAll("[data-mosaic-vision]");
        for (var i = 0; i < els.length; i++) {
            var el = els[i];
            var key = el.getAttribute("data-mosaic-vision");
            if (key === "range_cm") {
                el.value = isUnbounded ? "" : _fmt(vision.range_cm);
                el.disabled = isUnbounded;
            } else if (vision[key] !== undefined && vision[key] !== null) {
                el.value = _fmt(vision[key]);
            }
        }
        var unbounded = document.querySelector("[data-mosaic-vision-unbounded]");
        if (unbounded) {
            unbounded.checked = isUnbounded;
        }
    }

    // ----- public tools surface --------------------------------------------
    E.onPointerDown = onPointerDown;
    E.onPointerMove = onPointerMove;
    E.onPointerUp = onPointerUp;
    E.onKeyDown = onKeyDown;
    E.refreshProps = refreshProps;
    E.onPropChange = onPropChange;
    E.deleteSelected = deleteSelected;
    E.status = status;
    // Mosaic tab + vision-cone editing surface (T15). `initMosaicPanel` binds
    // every control's listeners ONCE (call from editor_boot.js's `boot()`);
    // `refreshMosaicPanel` re-populates the palette/grid/counts/cone-field DOM
    // from state (call after ingesting a loaded layout, or any time the state
    // was replaced wholesale rather than mutated in place).
    E.initMosaicPanel = initMosaicPanel;
    E.refreshMosaicPanel = refreshMosaicPanel;
    E.setMosaicMode = setMosaicMode;
})();
