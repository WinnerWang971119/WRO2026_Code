/* CTPS — Layout Editor: tools (Rect/Circle rubber-band + select/drag/delete +
 * Properties panel).
 *
 * The pointer-interaction + Properties layer of the two-tab offline editor
 * (object/cargo plan, T8 -- issue #57), built on editor_core.js's
 * `window.CTPS_EDITOR` namespace + its `unproject` (pixel->world) inverse:
 *
 *   - Field tab: Rect/Circle rubber-band draw (a new field object), click to
 *     select the topmost object, drag to move it, Delete/Backspace to remove it.
 *   - Robot tab: click a manipulator zone or drop point to select it, drag to
 *     move it, and (Rect tool) rubber-band to re-shape the selected zone -- all
 *     in the robot-LOCAL frame the Robot tab renders directly.
 *   - Properties panel wiring: fills + edits EVERY schema field for the current
 *     selection -- name / type (Tile|Cargo|Fixed) / color / x,y for both shapes,
 *     w,h + a heading rotation input for a rect (heading hidden for a circle),
 *     radius for a circle; for a robot selection only the geometric fields show.
 *
 * Vanilla ES5. Never sets an inline element style: show/hide is the `hidden`
 * property only; the canvas is the sole colour surface.
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
})();
