/* CTPS — Clear Trajectory Prediction Simulator: chrome (header + full-view + help).
 *
 * Owns the console's outer "chrome" — the parts that are not the canvas stage,
 * the rail, the transport bar, or the timeline dock:
 *
 *   1. Header wiring — the trace filename + source badge (from PLAYER_META),
 *      and the drift chip (worst drift_warn across the whole trace; click
 *      seeks to it; hidden when the trace has no drift events).
 *   2. The MAP / TRAIL / NODES layer chips — click toggles CTPS.store.layers;
 *      the active class always mirrors the store, including on load.
 *   3. Full-view (the on-canvas HUD full-screen is done here) — click toggles
 *      CTPS.store.fullView; the toggle drives the label/glyph on the switch,
 *      the `.is-fullview` class on the `.ctps-app` root (layout.css collapses
 *      the rail + timeline dock off that class), and the on-canvas HUD's
 *      visibility + current-command text. The HUD's holding chip is a
 *      reserved, statically-rendered shell (no cargo data yet, #57) — this
 *      file deliberately never touches it.
 *   4. `CTPS.help` — a `{ open, close }` pair backing the "? Help" button
 *      (and transport.js's `?` keymap handler) that shows a modal listing the
 *      keymap + the accent/hold/drift color legend.
 *
 * Like every sibling asset, this module only registers an `init(ctx)` onto
 * `CTPS._inits` at load time — boot.js invokes it (with `ctx = { store, dom,
 * track, meta }`) once every asset has finished registering. No DOM is
 * queried or mutated until then. Styling is class-based only: this file
 * never writes `el.style.<prop>`, only `classList` + `textContent` (and the
 * native `hidden` boolean attribute, which is not an inline style — it is
 * the same attribute the template already defaults these elements to). No
 * network, no frameworks, works offline. ES5 syntax throughout — this file
 * is inlined verbatim, not transpiled.
 */

"use strict";

(function () {
    var CTPS = window.CTPS;

    /**
     * Parse the drift distance in centimetres out of a `drift_warn` event
     * label of the form `"chassis 12.4 cm off node J6"` (track_builder.py's
     * exact format). Returns `null` when the label does not carry a parsable
     * "<number> cm" run — a defensive fallback, not an expected case.
     *
     * @param label the event's label string.
     * @return the parsed centimetre value, or null.
     */
    function parseDriftCm(label) {
        if (!label) {
            return null;
        }
        var match = /([0-9]*\.?[0-9]+)\s*cm/.exec(label);
        if (!match) {
            return null;
        }
        return parseFloat(match[1]);
    }

    /**
     * Scan `events` for the worst (largest centimetre) `drift_warn` event.
     *
     * @param events the full track event array (any order of types).
     * @return `{ t, cm }` for the worst drift event, or null when the trace
     *     carries no parsable drift_warn events at all.
     */
    function findWorstDrift(events) {
        var worst = null;
        for (var i = 0; i < events.length; i++) {
            var evt = events[i];
            if (!evt || evt.type !== "drift_warn") {
                continue;
            }
            var cm = parseDriftCm(evt.label);
            if (cm === null) {
                continue;
            }
            if (worst === null || cm > worst.cm) {
                worst = { t: evt.t, cm: cm };
            }
        }
        return worst;
    }

    /**
     * The label of the last event at-or-before `t` (the same "current
     * command" rule the rail's command panel and the full-view HUD both
     * follow), or an em dash placeholder before the first event / on an
     * empty track.
     *
     * @param engine `CTPS.engine` (closes over the ingested events array).
     * @param t the playhead time, in seconds.
     * @return the resolved label string.
     */
    function commandLabelAt(engine, t) {
        var idx = engine.findEventIndex(t);
        if (idx < 0) {
            return "—";
        }
        var evt = engine.events[idx];
        if (!evt) {
            return "—";
        }
        return evt.label || evt.type || "—";
    }

    /**
     * Build the (initially closed) help modal — the keymap + color legend —
     * and append it to `document.body`. Built once, entirely via
     * `document.createElement`/`textContent` (no `innerHTML`, no inline
     * SVG needed since unicode glyphs cover every icon here). Closes on a
     * backdrop click, the close button, or Escape.
     *
     * @param helpBtn the "? Help" button — focus returns to it on close.
     * @return `{ open, close }` — the pair exposed as `CTPS.help`.
     */
    function buildHelpModal(helpBtn) {
        var backdrop = document.createElement("div");
        backdrop.className = "ctps-modal";
        backdrop.setAttribute("role", "dialog");
        backdrop.setAttribute("aria-modal", "true");
        backdrop.setAttribute("aria-label", "Keymap and color legend");
        backdrop.hidden = true;

        var panel = document.createElement("div");
        panel.className = "ctps-modal-panel";
        backdrop.appendChild(panel);

        var header = document.createElement("div");
        header.className = "ctps-modal-header";
        panel.appendChild(header);

        var title = document.createElement("span");
        title.className = "ctps-modal-title";
        title.textContent = "Keymap & legend";
        header.appendChild(title);

        var closeBtn = document.createElement("button");
        closeBtn.type = "button";
        closeBtn.className = "ctps-modal-close";
        closeBtn.setAttribute("aria-label", "Close help");
        closeBtn.textContent = "✕";
        header.appendChild(closeBtn);

        var keySection = document.createElement("div");
        keySection.className = "ctps-modal-section";
        panel.appendChild(keySection);

        var keyTitle = document.createElement("div");
        keyTitle.className = "ctps-modal-section-title";
        keyTitle.textContent = "Keymap";
        keySection.appendChild(keyTitle);

        var keyList = document.createElement("div");
        keyList.className = "ctps-modal-keys";
        keySection.appendChild(keyList);

        // [key label, description] — the full keymap per the UI/UX spec.
        var KEYMAP = [
            ["Space", "Play / pause"],
            ["[  ]", "Speed down / up"],
            ["Shift+← / Shift+→", "Step to previous / next event"],
            ["Home / End", "Jump to start / end"],
            ["D", "Jump to next drift"],
            ["F", "Toggle full view"],
            ["?", "Show this help"],
        ];
        for (var i = 0; i < KEYMAP.length; i++) {
            var row = document.createElement("div");
            row.className = "ctps-modal-key-row";

            var kbd = document.createElement("span");
            kbd.className = "ctps-modal-key";
            kbd.textContent = KEYMAP[i][0];
            row.appendChild(kbd);

            var desc = document.createElement("span");
            desc.className = "ctps-modal-key-desc";
            desc.textContent = KEYMAP[i][1];
            row.appendChild(desc);

            keyList.appendChild(row);
        }

        var legendSection = document.createElement("div");
        legendSection.className = "ctps-modal-section";
        panel.appendChild(legendSection);

        var legendTitle = document.createElement("div");
        legendTitle.className = "ctps-modal-section-title";
        legendTitle.textContent = "Color legend";
        legendSection.appendChild(legendTitle);

        var legendList = document.createElement("div");
        legendList.className = "ctps-modal-legend";
        legendSection.appendChild(legendList);

        // [token modifier, description] — swatches resolve their fill from
        // the matching --ctps-* custom property in the stylesheet, never an
        // inline style.
        var LEGEND = [
            ["accent", "Playhead, robot, trail"],
            ["hold", "Cargo (reserved)"],
            ["drift", "Drift warning"],
        ];
        for (var j = 0; j < LEGEND.length; j++) {
            var lrow = document.createElement("div");
            lrow.className = "ctps-modal-legend-row";

            var swatch = document.createElement("span");
            swatch.className = "ctps-modal-swatch ctps-modal-swatch--" + LEGEND[j][0];
            swatch.setAttribute("aria-hidden", "true");
            lrow.appendChild(swatch);

            var ldesc = document.createElement("span");
            ldesc.className = "ctps-modal-legend-desc";
            ldesc.textContent = LEGEND[j][1];
            lrow.appendChild(ldesc);

            legendList.appendChild(lrow);
        }

        document.body.appendChild(backdrop);

        function close() {
            backdrop.hidden = true;
            if (helpBtn) {
                helpBtn.focus();
            }
        }

        function open() {
            backdrop.hidden = false;
            closeBtn.focus();
        }

        backdrop.addEventListener("click", function (ev) {
            // Only the backdrop itself counts as "outside the panel" — a
            // click that bubbled up from inside `panel` must not close it.
            if (ev.target === backdrop) {
                close();
            }
        });
        closeBtn.addEventListener("click", close);
        document.addEventListener("keydown", function (ev) {
            if (!backdrop.hidden && ev.key === "Escape") {
                close();
            }
        });

        return { open: open, close: close };
    }

    /**
     * Boot hook — pushed onto `CTPS._inits`, run once every sibling asset has
     * finished registering. Wires the header, the layer chips, full-view, and
     * the help modal; then subscribes once to `CTPS.store` to keep all of
     * them in sync with later state changes.
     *
     * @param ctx `{ store, dom, track, meta }` per the Contracts section.
     */
    function init(ctx) {
        var store = (ctx && ctx.store) || CTPS.store;
        var meta = (ctx && ctx.meta) || {};
        var engine = CTPS.engine;

        var appRoot = document.querySelector(".ctps-app");
        var filenameEl = document.getElementById("ctps-filename");
        var badgeEl = document.getElementById("ctps-source-badge");
        var driftChipEl = document.getElementById("ctps-drift-chip");
        var driftChipTextEl = document.getElementById("ctps-drift-chip-text");
        var helpBtn = document.getElementById("ctps-help");
        var fullviewBtn = document.getElementById("ctps-fullview");
        var fullviewGlyphEl = fullviewBtn
            ? fullviewBtn.querySelector(".ctps-fullview-glyph")
            : null;
        var fullviewTextEl = document.getElementById("ctps-fullview-text");
        var hudEl = document.getElementById("ctps-hud");
        var hudCommandEl = document.getElementById("ctps-hud-command");
        var layerChips = document.querySelectorAll(".ctps-layer-chips .ctps-chip");

        // ----- 1. header: filename + source badge ---------------------------
        if (filenameEl) {
            filenameEl.textContent = meta.filename || "";
        }
        if (badgeEl) {
            badgeEl.textContent = meta.source || "PREDICTED";
        }

        // ----- 1. header: drift chip ----------------------------------------
        // Drift events never change once the track is ingested, so the worst
        // one is computed exactly once here rather than on every refresh.
        var worstDrift = findWorstDrift(engine.events || []);
        if (worstDrift && driftChipEl) {
            if (driftChipTextEl) {
                driftChipTextEl.textContent =
                    "drift " + worstDrift.cm.toFixed(1) + " cm @ " +
                    worstDrift.t.toFixed(1) + " s";
            }
            // Toggled via the native `hidden` boolean attribute, matching the
            // template's own default state for this element — functionally
            // the "hidden class" the spec calls for, without depending on a
            // not-yet-authored stylesheet rule, and never `.style.<prop>=`.
            driftChipEl.hidden = false;
            driftChipEl.addEventListener("click", function () {
                store.seek(worstDrift.t);
            });
        } else if (driftChipEl) {
            driftChipEl.hidden = true;
        }

        // ----- 2. layer chips -------------------------------------------------
        function forEachLayerChip(fn) {
            for (var i = 0; i < layerChips.length; i++) {
                fn(layerChips[i]);
            }
        }

        forEachLayerChip(function (chip) {
            var name = chip.getAttribute("data-layer");
            if (!name) {
                return;
            }
            chip.addEventListener("click", function () {
                store.toggleLayer(name);
            });
        });

        function refreshLayerChips() {
            forEachLayerChip(function (chip) {
                var name = chip.getAttribute("data-layer");
                chip.classList.toggle("ctps-chip--active", !!store.layers[name]);
            });
        }
        refreshLayerChips();

        // ----- 3. full view ---------------------------------------------------
        function refreshHudCommand() {
            if (hudCommandEl) {
                hudCommandEl.textContent = commandLabelAt(engine, store.t);
            }
        }

        function refreshFullView() {
            if (appRoot) {
                appRoot.classList.toggle("is-fullview", !!store.fullView);
            }
            if (fullviewTextEl) {
                fullviewTextEl.textContent = store.fullView ? "Exit full view" : "Full";
            }
            if (fullviewGlyphEl) {
                fullviewGlyphEl.textContent = store.fullView ? "🗗" : "⛶";
            }
            if (fullviewBtn) {
                fullviewBtn.setAttribute("aria-pressed", store.fullView ? "true" : "false");
            }
            // The on-canvas HUD only ever shows in full view — same native
            // `hidden` toggle rationale as the drift chip above.
            if (hudEl) {
                hudEl.hidden = !store.fullView;
            }
            if (store.fullView) {
                refreshHudCommand();
            }
        }

        if (fullviewBtn) {
            fullviewBtn.addEventListener("click", function () {
                store.toggleFull();
            });
        }
        refreshFullView();

        // ----- state subscription ---------------------------------------------
        // One subscription drives every region this module owns; each region
        // only recomputes when its own store keys changed.
        store.subscribe(function (changedKeys, state) {
            if (changedKeys.indexOf("layers") !== -1) {
                refreshLayerChips();
            }
            if (changedKeys.indexOf("fullView") !== -1) {
                refreshFullView();
            }
            if (changedKeys.indexOf("t") !== -1 && state.fullView) {
                refreshHudCommand();
            }
        });

        // ----- 4. help modal ---------------------------------------------------
        CTPS.help = buildHelpModal(helpBtn);
        if (helpBtn) {
            helpBtn.addEventListener("click", function () {
                CTPS.help.open();
            });
        }
    }

    CTPS._inits.push(init);
})();
