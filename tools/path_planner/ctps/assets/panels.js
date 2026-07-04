/* CTPS — Clear Trajectory Prediction Simulator: right-rail panels.
 *
 * The right rail's THREE panels, all driven off `CTPS.store` and the pinned
 * seek engine — no rendering/canvas code lives here (that is renderer.js/T4):
 *
 *   1. Current command (`#ctps-command` / `#ctps-command-detail`) — the label
 *      of the last event at-or-before the playhead, refreshed on `["t"]`.
 *   2. Log / Inspector tabs — registered on `CTPS.panelTabs` (`log`,
 *      `inspector`) so a future tab (#23 telemetry) is an additive
 *      registration. The Log tab keeps its ~10-line window bounded via
 *      `CTPS.cursor()`'s forward/rebuild dispatch (forward appends the newly
 *      crossed lines; a backward seek rebuilds only the window, never a full
 *      `events` scan); each line seeks on click. The Inspector tab renders
 *      whatever `CTPS.inspectorFields` holds (populated by T8/selection.js) as
 *      a 2-column label/value grid, with a placeholder before T8 lands.
 *   3. Holding — the reserved #57 shell. Its dashed `.ctps-hold-empty` slots
 *      and the "0 / 3" count are already static markup in the HTML template
 *      (player_html.py); this file only asserts that static count explicitly
 *      and registers a no-op cursor consumer so the forward/rebuild seam
 *      exists end-to-end. No cargo data.
 *
 * Styling is class-driven only: this panel never touches `.style` (not even
 * `setProperty`) — every state it needs to convey (current/drift/active) is a
 * CSS modifier class for components.css (T13) to style. Vanilla ES5, no
 * frameworks, no network.
 */

"use strict";

(function () {
    var CTPS = window.CTPS;

    // ----- event -> command-panel / log-line text -------------------------

    //: Human prefix per event `type` (Data Model B). Unknown types fall back
    //: to the raw type string so a future event kind never renders blank.
    var COMMAND_PREFIXES = {
        goto: "goTo",
        drive: "Drive",
        manip: "Manipulator",
        wait: "Wait",
        print: "Print",
        beep: "Beep",
        led: "LED",
        task_start: "Task start",
        task_end: "Task end",
        drift_warn: "Drift",
    };

    function commandPrefix(type) {
        return Object.prototype.hasOwnProperty.call(COMMAND_PREFIXES, type)
            ? COMMAND_PREFIXES[type]
            : type;
    }

    function formatSeconds(t) {
        return t.toFixed(2) + "s";
    }

    /**
     * "<prefix>: <label>" (or just "<prefix>" when the event has no label).
     * Shared by the Current-command panel and every Log line so the two read
     * consistently.
     */
    function formatCommandText(ev) {
        var prefix = commandPrefix(ev.type);
        return ev.label ? (prefix + ": " + ev.label) : prefix;
    }

    /**
     * A detail line only when one is actually derivable from the event shape
     * — today that is exactly the `t_end` span a successful `goto` carries
     * (Data Model B). Everything else renders an empty detail line rather
     * than guessing.
     */
    function formatCommandDetail(ev) {
        if (ev && typeof ev.t_end === "number") {
            var span = ev.t_end - ev.t;
            return formatSeconds(ev.t) + " → " + formatSeconds(ev.t_end) +
                " (" + formatSeconds(span) + ")";
        }
        return "";
    }

    // ----- 1. Current command panel ----------------------------------------

    function renderCommandPanel(dom, idx) {
        if (!dom.command) {
            return;
        }
        if (idx === -1) {
            dom.command.textContent = "—";
            dom.command.classList.remove("ctps-command--drift");
            if (dom.commandDetail) {
                dom.commandDetail.textContent = "";
            }
            return;
        }
        var ev = CTPS.engine.events[idx];
        dom.command.textContent = formatCommandText(ev);
        dom.command.classList.toggle("ctps-command--drift", ev.type === "drift_warn");
        if (dom.commandDetail) {
            dom.commandDetail.textContent = formatCommandDetail(ev);
        }
    }

    // ----- 2a. Log tab: bounded ~10-line window via CTPS.cursor() ----------

    //: The Log window never holds more than this many lines, on either a
    //: forward-append or a backward-rebuild.
    var LOG_WINDOW_SIZE = 10;

    //: This module's own cursor position (see CTPS.cursor()'s step contract)
    //: plus the DOM elements currently in the window, oldest first (index 0)
    //: through newest/current (last index) — mirrors `#ctps-log`'s child order.
    var logPrevIdx = -1;
    var logLineEls = [];

    function makeLogLineEl(store, ev) {
        var el = document.createElement("div");
        el.className = "ctps-log-line" +
            (ev.type === "drift_warn" ? " ctps-log-line--drift" : "");
        el.textContent = formatSeconds(ev.t) + "  " + formatCommandText(ev);
        el.setAttribute("role", "button");
        el.tabIndex = 0;
        el.addEventListener("click", function () {
            store.seek(ev.t);
        });
        el.addEventListener("keydown", function (evt) {
            if (evt.key === "Enter" || evt.key === " ") {
                evt.preventDefault();
                store.seek(ev.t);
            }
        });
        return el;
    }

    function lastLogLineEl() {
        return logLineEls.length ? logLineEls[logLineEls.length - 1] : null;
    }

    function setLogLineCurrent(el, isCurrent) {
        if (el) {
            el.classList.toggle("ctps-log-line--current", isCurrent);
        }
    }

    /**
     * Forward branch: append the newly-crossed events, then trim the front
     * back down to the window size. A skipped-ahead slice (the Log wasn't
     * live through a big jump) is capped to the last LOG_WINDOW_SIZE entries
     * BEFORE touching the DOM, so this stays O(window) rather than
     * O(events crossed) — never a full re-render of everything skipped.
     */
    function appendLogLines(store, container, newEvents) {
        if (!container || newEvents.length === 0) {
            return;
        }
        if (newEvents.length > LOG_WINDOW_SIZE) {
            newEvents = newEvents.slice(newEvents.length - LOG_WINDOW_SIZE);
        }
        setLogLineCurrent(lastLogLineEl(), false);
        for (var i = 0; i < newEvents.length; i++) {
            var el = makeLogLineEl(store, newEvents[i]);
            container.appendChild(el);
            logLineEls.push(el);
        }
        while (logLineEls.length > LOG_WINDOW_SIZE) {
            var stale = logLineEls.shift();
            if (stale.parentNode === container) {
                container.removeChild(stale);
            }
        }
        setLogLineCurrent(lastLogLineEl(), true);
        container.scrollTop = container.scrollHeight;
    }

    /**
     * Rebuild branch (backward seek): replay only the bounded window ending
     * at `idx`, never the full `events` array.
     */
    function rebuildLogWindow(store, container, idx) {
        if (!container) {
            return;
        }
        container.textContent = "";
        logLineEls = [];
        if (idx < 0) {
            return;
        }
        var events = CTPS.engine.events;
        var start = Math.max(0, idx - LOG_WINDOW_SIZE + 1);
        for (var i = start; i <= idx; i++) {
            var el = makeLogLineEl(store, events[i]);
            container.appendChild(el);
            logLineEls.push(el);
        }
        setLogLineCurrent(lastLogLineEl(), true);
        container.scrollTop = container.scrollHeight;
    }

    /** One `CTPS.cursor()` step, dispatched to the forward/rebuild branches. */
    function stepLogCursor(store, container, idx) {
        CTPS.cursor().step(logPrevIdx, idx, {
            forward: function (newEvents) {
                appendLogLines(store, container, newEvents);
            },
            rebuild: function (newIdx) {
                rebuildLogWindow(store, container, newIdx);
            },
        });
        logPrevIdx = idx;
    }

    /** `panelTab` descriptor render(el, state) for `"log"`. */
    function renderLogTab(el, state) {
        var idx = CTPS.cursor().at(state.t);
        stepLogCursor(state, el, idx);
    }

    // ----- 2b. Inspector tab: renders CTPS.inspectorFields -----------------

    //: The inspector value's semantic tone comes from the field descriptor's
    //: tone(state) hint (T8), NOT a color: panels can only set CSS classes, so
    //: keying off a raw hex would couple three files to one literal token value.
    //: A tone of "drift" maps to the flagged modifier class.
    function inspectorValueClassName(tone) {
        return (tone === "drift")
            ? "ctps-inspector-value ctps-inspector-value--drift"
            : "ctps-inspector-value";
    }

    /** `panelTab` descriptor render(el, state) for `"inspector"`. */
    function renderInspectorTab(el, state) {
        if (!el) {
            return;
        }
        el.textContent = "";
        var fields = CTPS.inspectorFields.all();
        if (fields.length === 0) {
            var placeholder = document.createElement("div");
            placeholder.className = "ctps-inspector-empty";
            placeholder.textContent = "No inspector fields registered yet.";
            el.appendChild(placeholder);
            return;
        }
        var grid = document.createElement("div");
        grid.className = "ctps-inspector-grid";
        for (var i = 0; i < fields.length; i++) {
            var field = fields[i];

            var labelEl = document.createElement("div");
            labelEl.className = "ctps-inspector-label";
            labelEl.textContent = field.label;

            var valueText = "";
            try {
                valueText = field.value(state);
            } catch (valueErr) {
                valueText = "";
            }
            var tone = null;
            if (typeof field.tone === "function") {
                try {
                    tone = field.tone(state);
                } catch (toneErr) {
                    tone = null;
                }
            }
            var valueEl = document.createElement("div");
            valueEl.className = inspectorValueClassName(tone);
            valueEl.textContent = valueText;

            grid.appendChild(labelEl);
            grid.appendChild(valueEl);
        }
        el.appendChild(grid);
    }

    // ----- 3. Holding: reserved empty shell (#57 seam) ----------------------

    //: This module's own cursor position for the no-op Holding consumer —
    //: kept separate from `logPrevIdx` so the two windows step independently.
    var holdingPrevIdx = -1;

    /** Registered purely so the forward/rebuild seam exists end-to-end. */
    function stepHoldingCursor(idx) {
        CTPS.cursor().step(holdingPrevIdx, idx, {
            forward: function () {},
            rebuild: function () {},
        });
        holdingPrevIdx = idx;
    }

    /**
     * The dashed `.ctps-hold-empty` slots are already static markup in the
     * template (player_html.py) — nothing to build. This only asserts the
     * reserved "0 / 3" count explicitly, rather than leaning on the
     * template's default text staying in sync by accident. No cargo data.
     */
    function renderHoldingShell(dom) {
        if (dom.holdCount) {
            dom.holdCount.textContent = "0 / 3";
        }
    }

    // ----- Tabs: click -> store.setTab, activeTab -> active class + panel --

    function wireTabButton(buttonEl, store) {
        if (!buttonEl) {
            return;
        }
        buttonEl.addEventListener("click", function (evt) {
            store.setTab(evt.currentTarget.getAttribute("data-tab"));
        });
    }

    function wireTabButtons(dom, store) {
        wireTabButton(dom.tabLog, store);
        wireTabButton(dom.tabInspector, store);
    }

    /**
     * Toggle `.ctps-tab--active` (the accent underline is class-driven, in
     * components.css) + the panel's `hidden` attribute for one tab id, and —
     * only for the tab that just became active — render it through the
     * `panelTabs` registry so switching tabs always shows fresh content.
     */
    function setTabActive(buttonEl, panelEl, id, state) {
        var isActive = (state.activeTab === id);
        if (buttonEl) {
            buttonEl.classList.toggle("ctps-tab--active", isActive);
        }
        if (panelEl) {
            panelEl.hidden = !isActive;
        }
        if (isActive) {
            var descriptor = CTPS.panelTabs.get(id);
            if (descriptor) {
                descriptor.render(panelEl, state);
            }
        }
    }

    function applyActiveTab(dom, state) {
        setTabActive(dom.tabLog, dom.log, "log", state);
        setTabActive(dom.tabInspector, dom.inspector, "inspector", state);
    }

    // ----- boot -------------------------------------------------------------

    function queryDom() {
        return {
            command: document.getElementById("ctps-command"),
            commandDetail: document.getElementById("ctps-command-detail"),
            log: document.getElementById("ctps-log"),
            inspector: document.getElementById("ctps-inspector"),
            tabLog: document.getElementById("ctps-tab-log"),
            tabInspector: document.getElementById("ctps-tab-inspector"),
            holdCount: document.getElementById("ctps-hold-count"),
        };
    }

    /**
     * Wires the rail: registers the `log`/`inspector` panelTabs (so the
     * registry is meaningful for a future #23 tab too), wires tab-button
     * clicks + the Holding shell's static count, subscribes to the store
     * (one `findEventIndex`-backed `CTPS.cursor().at(t)` per `"t"` notify,
     * fed to the command panel, the Log cursor, AND the no-op Holding
     * cursor — "one bisect per refresh feeds every stateful panel"), then
     * runs an initial sync so the rail is populated before the first user
     * action.
     */
    function init(ctx) {
        var store = ctx.store;
        var dom = queryDom();

        CTPS.panelTabs.register({ id: "log", label: "Log", render: renderLogTab });
        CTPS.panelTabs.register({
            id: "inspector",
            label: "Inspector",
            render: renderInspectorTab,
        });

        wireTabButtons(dom, store);
        renderHoldingShell(dom);

        store.subscribe(function (changedKeys, state) {
            if (changedKeys.indexOf("t") !== -1) {
                var idx = CTPS.cursor().at(state.t);
                renderCommandPanel(dom, idx);
                stepLogCursor(state, dom.log, idx);
                stepHoldingCursor(idx);
                if (state.activeTab === "inspector") {
                    renderInspectorTab(dom.inspector, state);
                }
            }
            if (changedKeys.indexOf("selection") !== -1 && state.activeTab === "inspector") {
                renderInspectorTab(dom.inspector, state);
            }
            if (changedKeys.indexOf("activeTab") !== -1) {
                applyActiveTab(dom, state);
            }
        });

        // Initial sync: populate the rail for t=0 / the default active tab
        // without waiting for the first store action.
        var idx0 = CTPS.cursor().at(store.t);
        renderCommandPanel(dom, idx0);
        stepLogCursor(store, dom.log, idx0);
        stepHoldingCursor(idx0);
        applyActiveTab(dom, store);
    }

    CTPS._inits.push(init);
})();
