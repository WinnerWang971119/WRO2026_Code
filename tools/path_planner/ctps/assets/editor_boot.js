/* CTPS — Layout Editor: boot (wire the DOM + kick the first render).
 *
 * The one-time init for the three-tab offline editor (object/cargo plan, T8 --
 * issue #57; randomized-vision plan, T17 -- ADR-060). Inlined LAST, after
 * editor_core.js / editor_tools.js / editor_io.js have registered onto
 * `window.CTPS_EDITOR`, so every function it binds already exists. It:
 *
 *   - grabs the canvas + 2D context and every Properties-panel control into a
 *     shared `E.dom` map (the field/tab/tool/save/load ids from the emitter's
 *     body markup),
 *   - kicks off the optional inline field map (`E.initMap`),
 *   - binds the Field/Robot/Mosaic tabs (the Mosaic tab + the Robot-tab vision
 *     cone group per T14/T15), the Select/Rect/Circle tools, Save/Load + the
 *     hidden file input, the canvas pointer + Delete-key interactions, and a
 *     coalesced resize,
 *   - one-time binds every Mosaic-tab + vision-cone control via
 *     `E.initMosaicPanel()` (T15), and
 *   - runs the first `E.resize()` (which measures the stage, computes all three
 *     view transforms, and repaints) plus the initial `E.refreshProps()` and
 *     `E.refreshMosaicPanel()` (so the embedded-layout auto-load, which already
 *     ingests `mosaic`/`robot.vision` via `E.ingestLayout`, is reflected in the
 *     Mosaic-tab UI).
 *
 * Vanilla ES5. DOM show/hide + active state go through the `hidden` property and
 * classList only -- never an inline element style.
 */

"use strict";

(function () {
    var E = window.CTPS_EDITOR;
    if (!E) {
        return; // core failed to seed the namespace; nothing to wire
    }

    var ZOOM_WHEEL_FACTOR = 1.1; // multiplicative Robot-tab zoom step per wheel notch

    // One-click preset fills for the Properties Color field -- the WRO game-piece
    // palette. Single source of truth for both the swatch hex and its label; the
    // buttons are built from this list in _bindSwatches(). Hex is lower-case so
    // the active-swatch match against the <input type=color> value (always
    // lower-case) is a plain string compare.
    var PRESET_COLORS = [
        { hex: "#f2c200", label: "Yellow" },
        { hex: "#3f7fe0", label: "Blue" },
        { hex: "#ffffff", label: "White" },
        { hex: "#57b36b", label: "Green" },
        { hex: "#e23b3b", label: "Red" }
    ];

    // Built swatch <button>s, kept for _syncSwatchActive()'s highlight pass.
    var _swatchButtons = [];

    /** document.getElementById shorthand. */
    function $(id) {
        return document.getElementById(id);
    }

    /** The nearest ancestor (incl. self) matching a class, or null. */
    function _closest(el, cls) {
        if (!el) {
            return null;
        }
        return el.closest ? el.closest("." + cls) : null;
    }

    /** The `.ctps-editor-field` label wrapping an input (for show/hide). */
    function _fieldOf(el) {
        if (!el) {
            return null;
        }
        return el.closest ? el.closest(".ctps-editor-field") : el.parentNode;
    }

    /** Add/remove a class on an element (no-op when absent). */
    function _toggle(el, cls, on) {
        if (!el) {
            return;
        }
        if (on) {
            el.classList.add(cls);
        } else {
            el.classList.remove(cls);
        }
    }

    // ----- control bindings ------------------------------------------------

    /** Switch tabs + reflect the active tab in the tab buttons. */
    function _selectTab(name) {
        E.setTab(name);
        var d = E.dom;
        _toggle(d.tabField, "ctps-editor-tab--active", name === "field");
        _toggle(d.tabRobot, "ctps-editor-tab--active", name === "robot");
        _toggle(d.tabMosaic, "ctps-editor-tab--active", name === "mosaic");
        // The render-size (zoom) control and the vision-cone group apply to the
        // Robot tab only; the Mosaic panel applies to the Mosaic tab only.
        if (d.zoom) {
            d.zoom.hidden = name !== "robot";
        }
        if (d.robotVision) {
            d.robotVision.hidden = name !== "robot";
        }
        if (d.mosaicPanel) {
            d.mosaicPanel.hidden = name !== "mosaic";
        }
        E.refreshProps();
    }

    /** Reflect a zoom multiplier onto the slider position + the "%" readout. */
    function _reflectZoom(zoom) {
        var d = E.dom;
        var pct = Math.round(zoom * 100);
        if (d.zoomRange) {
            d.zoomRange.value = String(pct);
        }
        if (d.zoomReadout) {
            d.zoomReadout.textContent = pct + "%";
        }
    }

    /** Slider input -> set the render zoom, then reflect the clamped value back. */
    function _onZoomInput() {
        var d = E.dom;
        if (!d.zoomRange) {
            return;
        }
        var pct = parseFloat(d.zoomRange.value);
        _reflectZoom(E.setRobotZoom(pct / 100));
    }

    /** Mouse-wheel over the canvas zooms the Robot tab (up = larger, down = smaller). */
    function _onCanvasWheel(ev) {
        if (E.state.activeTab !== "robot" || ev.deltaY === 0) {
            return;
        }
        ev.preventDefault(); // keep the page from scrolling while zooming
        var factor = ev.deltaY < 0 ? ZOOM_WHEEL_FACTOR : 1 / ZOOM_WHEEL_FACTOR;
        _reflectZoom(E.setRobotZoom(E.state.robotZoom * factor));
    }

    /** Set the active tool + reflect it in the tool buttons and canvas cursor. */
    function _selectTool(name) {
        E.setTool(name);
        var d = E.dom;
        _toggle(d.tools.select, "ctps-editor-tool--active", name === "select");
        _toggle(d.tools.rect, "ctps-editor-tool--active", name === "rect");
        _toggle(d.tools.circle, "ctps-editor-tool--active", name === "circle");
        _toggle(d.stage, "is-draw", name !== "select");
    }

    function _bindTabs() {
        var d = E.dom;
        if (d.tabField) {
            d.tabField.addEventListener("click", function () { _selectTab("field"); });
        }
        if (d.tabRobot) {
            d.tabRobot.addEventListener("click", function () { _selectTab("robot"); });
        }
        if (d.tabMosaic) {
            d.tabMosaic.addEventListener("click", function () { _selectTab("mosaic"); });
        }
    }

    function _bindTools() {
        var d = E.dom;
        var names = ["select", "rect", "circle"];
        for (var i = 0; i < names.length; i++) {
            (function (name) {
                var btn = d.tools[name];
                if (btn) {
                    btn.addEventListener("click", function () { _selectTool(name); });
                }
            })(names[i]);
        }
    }

    function _bindProps() {
        var d = E.dom;
        var inputs = [d.name, d.type, d.color, d.x, d.y, d.w, d.h, d.heading, d.radius];
        for (var i = 0; i < inputs.length; i++) {
            var el = inputs[i];
            if (el) {
                el.addEventListener("input", E.onPropChange);
                el.addEventListener("change", E.onPropChange);
            }
        }
        if (d.del) {
            d.del.addEventListener("click", function () { E.deleteSelected(); });
        }
    }

    /** Apply a preset hex to the selection: mirror it into the native colour
     *  input, run the shared prop-change path, and refresh the active ring. */
    function _applyPreset(hex) {
        var d = E.dom;
        if (!d.color) {
            return;
        }
        d.color.value = hex;
        E.onPropChange(); // reads d.color.value -> writes the object + repaints
        _syncSwatchActive();
    }

    /** Build the preset swatch buttons from PRESET_COLORS and wire their clicks.
     *  Each fill rides on `--ctps-swatch-color` (setProperty, the ADR-053 carve-
     *  out) so no bare hex or inline style attribute ever lands in the markup. */
    function _bindSwatches() {
        var host = $("ctps-editor-swatches");
        if (!host) {
            return;
        }
        _swatchButtons = [];
        for (var i = 0; i < PRESET_COLORS.length; i++) {
            (function (preset) {
                var btn = document.createElement("button");
                btn.type = "button";
                btn.className = "ctps-editor-swatch";
                btn.title = preset.label;
                btn.setAttribute("aria-label", preset.label);
                btn.dataset.color = preset.hex;
                btn.style.setProperty("--ctps-swatch-color", preset.hex);
                btn.addEventListener("click", function () {
                    _applyPreset(preset.hex);
                });
                host.appendChild(btn);
                _swatchButtons.push(btn);
            })(PRESET_COLORS[i]);
        }
    }

    /** Highlight the swatch whose hex matches the current colour, if any. */
    function _syncSwatchActive() {
        var cur = (E.dom && E.dom.color) ? String(E.dom.color.value).toLowerCase() : "";
        for (var i = 0; i < _swatchButtons.length; i++) {
            var btn = _swatchButtons[i];
            _toggle(btn, "ctps-editor-swatch--active", btn.dataset.color === cur);
        }
    }

    function _bindIO() {
        var d = E.dom;
        if (d.save) {
            d.save.addEventListener("click", function () { E.save(); });
        }
        if (d.loadBtn) {
            d.loadBtn.addEventListener("click", function () {
                if (d.file) {
                    d.file.click();
                }
            });
        }
        if (d.file) {
            d.file.addEventListener("change", function (ev) {
                var f = ev.target.files && ev.target.files[0];
                E.handleFile(f);
                ev.target.value = ""; // allow re-selecting the same file
            });
        }
    }

    function _bindCanvas() {
        var c = E.canvas;
        var pointer = !!window.PointerEvent;
        var down = pointer ? "pointerdown" : "mousedown";
        var move = pointer ? "pointermove" : "mousemove";
        var up = pointer ? "pointerup" : "mouseup";
        c.addEventListener(down, E.onPointerDown);
        // Move/up on window so a drag continues (and ends) off the canvas.
        window.addEventListener(move, E.onPointerMove);
        window.addEventListener(up, E.onPointerUp);
        window.addEventListener("keydown", E.onKeyDown);
        // Non-passive so the Robot-tab wheel zoom can preventDefault the scroll.
        c.addEventListener("wheel", _onCanvasWheel, { passive: false });
    }

    function _bindZoom() {
        var d = E.dom;
        if (d.zoomRange) {
            d.zoomRange.addEventListener("input", _onZoomInput);
            d.zoomRange.addEventListener("change", _onZoomInput);
        }
    }

    function _bindResize() {
        var queued = false;
        function sched() {
            if (queued) {
                return;
            }
            queued = true;
            window.requestAnimationFrame(function () {
                queued = false;
                E.resize();
            });
        }
        if (typeof ResizeObserver === "function") {
            new ResizeObserver(sched).observe(E.canvas);
        }
        window.addEventListener("resize", sched);
    }

    // ----- init ------------------------------------------------------------

    function boot() {
        var canvas = $("ctps-editor-canvas");
        if (!canvas) {
            return;
        }
        var ctx = canvas.getContext("2d");
        if (!ctx) {
            return;
        }
        E.canvas = canvas;
        E.ctx = ctx;

        var name = $("ctps-editor-prop-name");
        var type = $("ctps-editor-prop-type");
        var color = $("ctps-editor-prop-color");
        var heading = $("ctps-editor-prop-heading");
        E.dom = {
            canvas: canvas,
            ctx: ctx,
            stage: _closest(canvas, "ctps-editor-stage"),
            status: $("ctps-editor-status"),
            tabField: $("ctps-editor-tab-field"),
            tabRobot: $("ctps-editor-tab-robot"),
            tabMosaic: $("ctps-editor-tab-mosaic"),
            robotVision: $("ctps-editor-robot-vision"),
            mosaicPanel: $("ctps-editor-mosaic-panel"),
            zoom: $("ctps-editor-zoom"),
            zoomRange: $("ctps-editor-zoom-range"),
            zoomReadout: $("ctps-editor-zoom-readout"),
            tools: {
                select: $("ctps-tool-select"),
                rect: $("ctps-tool-rect"),
                circle: $("ctps-tool-circle"),
            },
            save: $("ctps-editor-save"),
            saveState: $("ctps-editor-save-state"),
            loadBtn: $("ctps-editor-load-btn"),
            file: $("ctps-editor-file"),
            propsEmpty: $("ctps-editor-props-empty"),
            propsFields: $("ctps-editor-props-fields"),
            name: name,
            nameField: _fieldOf(name),
            type: type,
            typeField: _fieldOf(type),
            color: color,
            colorField: _fieldOf(color),
            x: $("ctps-editor-prop-x"),
            y: $("ctps-editor-prop-y"),
            rectFields: $("ctps-editor-fields-rect"),
            w: $("ctps-editor-prop-w"),
            h: $("ctps-editor-prop-h"),
            heading: heading,
            headingField: _fieldOf(heading),
            circleFields: $("ctps-editor-fields-circle"),
            radius: $("ctps-editor-prop-radius"),
            del: $("ctps-editor-delete"),
        };

        _bindTabs();
        _bindTools();
        _bindProps();
        _bindSwatches();
        _bindIO();
        _bindCanvas();
        _bindZoom();
        _bindResize();

        // Let refreshProps() (editor_tools.js) re-light the active swatch when the
        // selection changes, without that file reaching into this closure.
        E.syncSwatchActive = _syncSwatchActive;

        // One-time binding of every Mosaic-tab + vision-cone control (T15). Safe
        // to call before the embedded-layout ingest below: it binds listeners
        // and does an initial (soon-to-be-stale) populate from the seeded
        // defaults, which the refreshMosaicPanel() call after ingest corrects.
        if (E.initMosaicPanel) {
            E.initMosaicPanel();
        }

        E.initMap();
        // Auto-load the embedded `const LAYOUT` (the recently-saved objects +
        // zones + T16's mosaic/robot.vision blocks) so the editor opens on the
        // last layout, not a fresh scene. Runs BEFORE initSaveState so the
        // loaded scene is the clean baseline.
        if (typeof LAYOUT !== "undefined" && LAYOUT && E.ingestLayout) {
            E.ingestLayout(LAYOUT);
        }
        E.resize();
        E.refreshProps();
        // Re-populate the Mosaic-tab palette/grid/counts/cone fields from
        // whatever E.state.mosaic/vision now holds post-ingest, so a loaded
        // layout's mosaic/vision reflects in the UI rather than the pre-ingest
        // defaults initMosaicPanel() populated it with above.
        if (E.refreshMosaicPanel) {
            E.refreshMosaicPanel();
        }
        if (E.initSaveState) {
            E.initSaveState();
        }
        // Sync the zoom control to the core's default multiplier (it starts
        // hidden on the Field tab; the value/readout are ready for the Robot tab).
        _reflectZoom(E.state.robotZoom);
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", boot);
    } else {
        boot();
    }
})();
