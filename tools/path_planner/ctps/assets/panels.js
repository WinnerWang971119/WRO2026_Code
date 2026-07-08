/* CTPS — Clear Trajectory Prediction Simulator: right-rail panels.
 *
 * The right rail's THREE panels, all driven off `CTPS.store` and the pinned
 * seek engine — no rendering/canvas code lives here (that is renderer.js/T4):
 *
 *   1. Current command (`#ctps-command` / `#ctps-command-detail`) — the label
 *      of the last event at-or-before the playhead, refreshed on `["t"]`.
 *   2. Log / Inspector tabs — registered on `CTPS.panelTabs` (`log`,
 *      `inspector`) so a future tab (#23 telemetry) is an additive
 *      registration. The Log tab keeps full history for true scrollback;
 *      forward seek appends newly crossed lines and backward seek rebuilds
 *      the log up to the current cursor index; each line seeks on click.
 *      The Inspector tab renders
 *      whatever `CTPS.inspectorFields` holds (populated by T8/selection.js) as
 *      a 2-column label/value grid, with a placeholder before T8 lands.
 *   3. Cargo (issue #57) — the two-tab Magazine/Inventory panel that replaced
 *      the reserved Holding shell. This file owns only the tab-switching
 *      CHROME: which of the two tab buttons carries `.ctps-cargo-tab--active`
 *      and which of the two `.ctps-cargo-panel` divs is visible, driven by
 *      `store.activeCargoTab` -- the same idiom as the Log/Inspector tabs
 *      above, but its own rail section (not registered via `panelTabs`). The
 *      panel CONTENT (the magazine grid, the inventory hold card, the tab
 *      count badges, the stage HUD line) and the tab-click wiring are
 *      cargo.js's (`store.scene`/`store.objects` are its domain, not this
 *      file's).
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

    // ----- 2a. Log tab: full scrollback via CTPS.cursor() -------------------

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
     * Forward branch: append every newly-crossed event so the operator keeps
     * full scrollback history.
     */
    function appendLogLines(store, container, newEvents) {
        if (!container || newEvents.length === 0) {
            return;
        }
        setLogLineCurrent(lastLogLineEl(), false);
        for (var i = 0; i < newEvents.length; i++) {
            var el = makeLogLineEl(store, newEvents[i]);
            container.appendChild(el);
            logLineEls.push(el);
        }
        setLogLineCurrent(lastLogLineEl(), true);
        container.scrollTop = container.scrollHeight;
    }

    /**
     * Rebuild branch (backward seek): reconcile the existing line list down to
     * `idx`, keeping full scrollback complete at every cursor position. Trims the
     * surplus off the END rather than wiping and replaying the whole prefix, so a
     * backward scrub over a long mission is O(lines removed), not O(prefix) per
     * frame (which janked/froze the UI on hundreds of events).
     */
    function rebuildLogWindow(store, container, idx) {
        if (!container) {
            return;
        }
        var target = (idx < 0) ? 0 : idx + 1; // number of lines that should show
        while (logLineEls.length > target) {
            var removed = logLineEls.pop();
            if (removed && removed.parentNode) {
                removed.parentNode.removeChild(removed);
            }
        }
        // Backfill any missing tail so the window always covers 0..idx (defensive;
        // forward crossings normally flow through `appendLogLines`).
        var events = CTPS.engine.events;
        for (var i = logLineEls.length; i < target; i++) {
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

    // ----- 3. Cargo tab-switching shell (#57 seam) --------------------------

    /**
     * Toggle `.ctps-cargo-tab--active` on the two cargo tab buttons and
     * `hidden` on the two `.ctps-cargo-panel` divs for `state.activeCargoTab`
     * — the same active-class/hidden-attribute mechanic as
     * `setTabActive`/`applyActiveTab` above, kept as its own function (rather
     * than folded into that pair) because the cargo panel is its own rail
     * section, not a `panelTabs` registrant. Content inside the panels is
     * cargo.js's; this only decides which one is on screen.
     */
    function renderHoldingShell(dom, state) {
        var activeTab = (state && state.activeCargoTab) || "magazine";
        var isMagazine = (activeTab === "magazine");
        if (dom.cargoTabMagazine) {
            dom.cargoTabMagazine.classList.toggle("ctps-cargo-tab--active", isMagazine);
        }
        if (dom.cargoTabInventory) {
            dom.cargoTabInventory.classList.toggle("ctps-cargo-tab--active", !isMagazine);
        }
        if (dom.cargoPanelMagazine) {
            dom.cargoPanelMagazine.hidden = !isMagazine;
        }
        if (dom.cargoPanelInventory) {
            dom.cargoPanelInventory.hidden = isMagazine;
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
            cargoTabMagazine: document.getElementById("ctps-cargo-tab-magazine"),
            cargoTabInventory: document.getElementById("ctps-cargo-tab-inventory"),
            cargoPanelMagazine: document.getElementById("ctps-cargo-panel-magazine"),
            cargoPanelInventory: document.getElementById("ctps-cargo-panel-inventory"),
        };
    }

    /**
     * Wires the rail: registers the `log`/`inspector` panelTabs (so the
     * registry is meaningful for a future #23 tab too), wires tab-button
     * clicks + the cargo tab-switching shell, subscribes to the store (one
     * `findEventIndex`-backed `CTPS.cursor().at(t)` per `"t"` notify, fed to
     * the command panel and the Log cursor; `activeCargoTab` re-runs the
     * cargo shell), then runs an initial sync so the rail is populated
     * before the first user action.
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
        renderHoldingShell(dom, store);

        store.subscribe(function (changedKeys, state) {
            if (changedKeys.indexOf("t") !== -1) {
                var idx = CTPS.cursor().at(state.t);
                renderCommandPanel(dom, idx);
                stepLogCursor(state, dom.log, idx);
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
            if (changedKeys.indexOf("activeCargoTab") !== -1) {
                renderHoldingShell(dom, state);
            }
        });

        // Initial sync: populate the rail for t=0 / the default active tab
        // without waiting for the first store action.
        var idx0 = CTPS.cursor().at(store.t);
        renderCommandPanel(dom, idx0);
        stepLogCursor(store, dom.log, idx0);
        applyActiveTab(dom, store);
    }

    CTPS._inits.push(init);
})();
