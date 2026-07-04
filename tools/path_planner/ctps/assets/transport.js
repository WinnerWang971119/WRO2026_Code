/* CTPS — Clear Trajectory Prediction Simulator: transport bar + keymap.
 *
 * Wires the transport bar (play/pause, seek, time readout, speed, home/end,
 * drift jumps) and the global keymap to `CTPS.store` (T3). This file owns two
 * things and nothing else:
 *
 *   1. The playback clock: a requestAnimationFrame loop that advances
 *      `store.t` by real elapsed time times `store.speed` while
 *      `store.playing`, stopping at `duration_s`. It reuses the original
 *      pre-refactor player.js clock's timing math (per-frame delta, restart-
 *      from-start on resume-at-end) but drives it entirely through store
 *      actions so every other module reacts via `store.subscribe`.
 *   2. DOM wiring for `#ctps-play` / `#ctps-seek` / `#ctps-time` / the speed
 *      segmented control / `#ctps-home` / `#ctps-end` / the drift-jump
 *      buttons, plus the window-level keymap (Space, `[`/`]`, Shift+Left/
 *      Right, Home/End, `D`, `F`, `?`).
 *
 * Pure transport: no canvas, no timeline, no panels — those live in the
 * sibling render/timeline/panel assets. The only dynamic style write in this
 * file is `document.documentElement.style.setProperty('--ctps-frac', ...)`
 * (the ADR-053 carve-out); every other visual state change goes through a
 * DOM property (`.value`, `.textContent`) or `classList`, never `.style.*`.
 * Concatenated into its own <script> AFTER player.js + store.js (so
 * `window.CTPS`, `CTPS.engine` and `CTPS.store` already exist) and BEFORE
 * boot.js, which runs this file's `init(ctx)` on DOMContentLoaded. Vanilla JS
 * only — no frameworks, no network, works offline.
 */

"use strict";

(function () {
    var CTPS = window.CTPS;

    // ----- module state (assigned in init(ctx)) ---------------------------
    var store = null;

    var playBtn = null;
    var homeBtn = null;
    var endBtn = null;
    var driftPrevBtn = null;
    var driftNextBtn = null;
    var seekInput = null;
    var timeEl = null;
    var speedContainer = null;

    // ----- playback clock ---------------------------------------------------
    var rafHandle = null;
    var lastFrameTs = null;

    // The four speed presets the segmented control and `[`/`]` cycle through,
    // slowest to fastest.
    var SPEED_STEPS = [0.5, 1, 2, 4];

    /**
     * The track's total duration in seconds, or 0 when no track (or no
     * duration) was ingested. `CTPS.engine.track` is null when the page has no
     * inline TRACK (see player.js), so this guards that case the same way
     * store.js's private `duration()` helper does.
     */
    function trackDuration() {
        return (CTPS.engine.track && CTPS.engine.track.duration_s) || 0;
    }

    /** Format seconds with 2 decimal places, guarding non-finite input. */
    function formatTime(t) {
        var v = (typeof t === "number" && isFinite(t)) ? t : 0;
        return v.toFixed(2);
    }

    /**
     * Reflect the playhead time `t` onto the seek input, the `--ctps-frac`
     * custom property (read by the seek fill overlay and the timeline
     * playhead), and the time readout. Called on every store `t` change and
     * once at init to sync the real duration into the placeholder readout.
     */
    function reflectSeek(t) {
        var duration = trackDuration();
        var frac = (duration > 0) ? (t / duration) : 0;
        if (frac < 0) {
            frac = 0;
        } else if (frac > 1) {
            frac = 1;
        }
        if (seekInput) {
            seekInput.value = String(Math.round(frac * 1000));
        }
        document.documentElement.style.setProperty("--ctps-frac", frac);
        if (timeEl) {
            timeEl.textContent = formatTime(t) + " / " + formatTime(duration) + " s";
        }
    }

    /** Toggle the active speed segment to match `speed`. */
    function reflectSpeed(speed) {
        if (!speedContainer) {
            return;
        }
        var buttons = speedContainer.querySelectorAll(".ctps-speed-btn");
        for (var i = 0; i < buttons.length; i++) {
            var btnSpeed = parseFloat(buttons[i].getAttribute("data-speed"));
            if (btnSpeed === speed) {
                buttons[i].classList.add("ctps-speed-btn--active");
            } else {
                buttons[i].classList.remove("ctps-speed-btn--active");
            }
        }
    }

    /** Swap the play button glyph between play (unicode BLACK RIGHT-POINTING
     * TRIANGLE) and pause (two unicode HEAVY VERTICAL BAR glyphs). */
    function updatePlayGlyph(playing) {
        if (!playBtn) {
            return;
        }
        playBtn.textContent = playing ? "❚❚" : "▶";
    }

    // ----- playback clock ---------------------------------------------------

    /** Cancel any pending animation frame and clear the per-frame baseline. */
    function stopClock() {
        if (rafHandle !== null) {
            window.cancelAnimationFrame(rafHandle);
            rafHandle = null;
        }
        lastFrameTs = null;
    }

    /**
     * requestAnimationFrame loop. Advances `store.t` by real elapsed time
     * times `store.speed`, stopping (and pausing) at `duration_s`. Every
     * advance goes through `store.setT` so `reflectSeek` (subscribed on `t`)
     * stays the single place the seek UI is synced.
     */
    function tick(nowMs) {
        if (!store.playing) {
            rafHandle = null;
            lastFrameTs = null;
            return;
        }
        if (lastFrameTs === null) {
            lastFrameTs = nowMs;
        }
        var dtSeconds = (nowMs - lastFrameTs) / 1000;
        lastFrameTs = nowMs;
        var duration = trackDuration();
        var next = store.t + dtSeconds * store.speed;
        if (next >= duration) {
            store.setT(duration);
            store.setPlaying(false);
            return;
        }
        store.setT(next);
        rafHandle = window.requestAnimationFrame(tick);
    }

    /** Kick off the clock loop if it is not already running. */
    function startClock() {
        if (rafHandle === null) {
            lastFrameTs = null;
            rafHandle = window.requestAnimationFrame(tick);
        }
    }

    /**
     * React to a `playing` change: swap the glyph, and start/stop the clock.
     * On resume from a fully-played track, restarts from the beginning first
     * (mirrors the pre-refactor player's "replay" behavior) — guarded against
     * a zero-duration (empty) track so it cannot loop on nothing.
     */
    function handlePlayingChange(playing) {
        updatePlayGlyph(playing);
        if (playing) {
            var duration = trackDuration();
            if (duration > 0 && store.t >= duration) {
                store.seek(0);
            }
            startClock();
        } else {
            stopClock();
        }
    }

    // ----- home / end ---------------------------------------------------

    function jumpToStart() {
        store.seek(0);
    }

    function jumpToEnd() {
        store.seek(trackDuration());
    }

    // ----- drift jumps (bisect over drift_warn events) ----------------------

    /** Return the `t` of every `drift_warn` event, in track order (sorted). */
    function driftTimes() {
        var events = (CTPS.engine && CTPS.engine.events) || [];
        var times = [];
        for (var i = 0; i < events.length; i++) {
            if (events[i].type === "drift_warn") {
                times.push(events[i].t);
            }
        }
        return times;
    }

    /** Bounded binary search: the smallest time in the sorted `times` that is
     * strictly greater than `t`, or null if none. */
    function bisectAfter(times, t) {
        var lo = 0;
        var hi = times.length;
        while (lo < hi) {
            var mid = lo + Math.floor((hi - lo) / 2);
            if (times[mid] > t) {
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        return (lo < times.length) ? times[lo] : null;
    }

    /** Bounded binary search: the largest time in the sorted `times` that is
     * strictly less than `t`, or null if none. */
    function bisectBefore(times, t) {
        var lo = 0;
        var hi = times.length;
        while (lo < hi) {
            var mid = lo + Math.floor((hi - lo) / 2);
            if (times[mid] < t) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return (lo > 0) ? times[lo - 1] : null;
    }

    /** Seek to the previous (`direction < 0`) or next (`direction > 0`) drift
     * warning relative to the current playhead. No-op if there is none. */
    function jumpDrift(direction) {
        var times = driftTimes();
        if (times.length === 0) {
            return;
        }
        var target = (direction > 0)
            ? bisectAfter(times, store.t)
            : bisectBefore(times, store.t);
        if (target !== null) {
            store.seek(target);
        }
    }

    // ----- event stepping (Shift+Left / Shift+Right) -----------------------

    // Tolerance for "the playhead is sitting exactly on this event" versus
    // "the playhead is strictly past it" — guards float drift from the clock.
    var EVENT_EPSILON = 1e-6;

    /** Step the playhead to the previous (`direction < 0`) or next
     * (`direction > 0`) event, via the pinned `findEventIndex` bisect. */
    function stepEvent(direction) {
        var events = CTPS.engine.events || [];
        if (events.length === 0) {
            return;
        }
        var idx = CTPS.engine.findEventIndex(store.t);
        var targetIdx;
        if (direction > 0) {
            targetIdx = idx + 1;
            if (targetIdx >= events.length) {
                return;
            }
        } else {
            if (idx < 0) {
                return;
            }
            // Sitting strictly past events[idx].t: that event itself is
            // "previous". Sitting on (or before) it: step back one further.
            targetIdx = (store.t > events[idx].t + EVENT_EPSILON) ? idx : idx - 1;
            if (targetIdx < 0) {
                return;
            }
        }
        store.seek(events[targetIdx].t);
    }

    // ----- speed cycling (`[` / `]`) ----------------------------------------

    /** Index into SPEED_STEPS nearest the store's current speed. */
    function currentSpeedIndex() {
        var speed = store.speed;
        var bestIdx = 0;
        var bestDiff = Infinity;
        for (var i = 0; i < SPEED_STEPS.length; i++) {
            var diff = Math.abs(SPEED_STEPS[i] - speed);
            if (diff < bestDiff) {
                bestDiff = diff;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    /** Step one preset up (`direction > 0`) or down (`direction < 0`) through
     * SPEED_STEPS, clamped at either end (never wraps). */
    function cycleSpeed(direction) {
        var idx = currentSpeedIndex() + direction;
        if (idx < 0) {
            idx = 0;
        } else if (idx > SPEED_STEPS.length - 1) {
            idx = SPEED_STEPS.length - 1;
        }
        store.setSpeed(SPEED_STEPS[idx]);
    }

    // ----- help ("?") --------------------------------------------------

    /** Dispatch to T9's help modal if it has registered itself; else no-op —
     * the keymap must not throw just because chrome.js has not loaded. */
    function openHelp() {
        if (window.CTPS.help && typeof window.CTPS.help.open === "function") {
            window.CTPS.help.open();
        }
    }

    // ----- keymap ------------------------------------------------------

    /** True when the focused element is a free-text control the keymap must
     * not hijack (there are none in this UI today; guarded for future-proofing
     * per the transport spec). The native range input is deliberately NOT
     * treated as a text input — its own arrow-key stepping must keep working. */
    function isTextInputFocused() {
        var el = document.activeElement;
        if (!el) {
            return false;
        }
        if (el.isContentEditable) {
            return true;
        }
        var tag = el.tagName;
        if (tag === "TEXTAREA" || tag === "SELECT") {
            return true;
        }
        if (tag === "INPUT") {
            var type = (el.getAttribute("type") || "text").toLowerCase();
            return type !== "range" && type !== "button" && type !== "checkbox" &&
                type !== "radio" && type !== "submit";
        }
        return false;
    }

    /**
     * Window-level keydown handler. Plain ArrowLeft/ArrowRight are left alone
     * so the browser's native `<input type=range>` frame-step still works when
     * `#ctps-seek` has focus; every other bound key calls `preventDefault()`
     * (this also cancels the browser's Space-activates-focused-button default
     * action, so clicking `#ctps-play` then pressing Space cannot double-fire
     * `togglePlay`). Ctrl/Meta/Alt combinations are left untouched so OS/
     * browser shortcuts (Cmd+F, Ctrl+Space, ...) keep working.
     */
    function onKeyDown(evt) {
        if (isTextInputFocused()) {
            return;
        }
        if (evt.ctrlKey || evt.metaKey || evt.altKey) {
            return;
        }
        var key = evt.key;
        if (key === "ArrowLeft" || key === "ArrowRight") {
            if (evt.shiftKey) {
                evt.preventDefault();
                stepEvent(key === "ArrowRight" ? 1 : -1);
            }
            return;
        }
        switch (key) {
            case " ":
            case "Spacebar":
                evt.preventDefault();
                store.togglePlay();
                break;
            case "[":
                evt.preventDefault();
                cycleSpeed(-1);
                break;
            case "]":
                evt.preventDefault();
                cycleSpeed(1);
                break;
            case "Home":
                evt.preventDefault();
                jumpToStart();
                break;
            case "End":
                evt.preventDefault();
                jumpToEnd();
                break;
            case "?":
                evt.preventDefault();
                openHelp();
                break;
            default:
                var lower = (typeof key === "string") ? key.toLowerCase() : "";
                if (lower === "d") {
                    evt.preventDefault();
                    jumpDrift(1);
                } else if (lower === "f") {
                    evt.preventDefault();
                    store.toggleFull();
                }
                break;
        }
    }

    // ----- wiring ------------------------------------------------------

    /**
     * Bind every transport control to `ctx.store` and sync the DOM to the
     * store's current values (the static HTML markup ships zeroed
     * placeholders; this replaces them with the real duration/speed/playing
     * state the moment the page boots). Registered via `CTPS._inits` — see
     * the module header.
     */
    function init(ctx) {
        store = ctx.store;

        playBtn = document.getElementById("ctps-play");
        homeBtn = document.getElementById("ctps-home");
        endBtn = document.getElementById("ctps-end");
        driftPrevBtn = document.getElementById("ctps-drift-prev");
        driftNextBtn = document.getElementById("ctps-drift-next");
        seekInput = document.getElementById("ctps-seek");
        timeEl = document.getElementById("ctps-time");
        speedContainer = document.querySelector(".ctps-speed");

        if (playBtn) {
            playBtn.addEventListener("click", function () {
                store.togglePlay();
            });
        }

        if (seekInput) {
            seekInput.addEventListener("input", function () {
                var frac = Number(seekInput.value) / 1000;
                if (store.playing) {
                    store.setPlaying(false);
                }
                store.seek(frac * trackDuration());
            });
        }

        if (homeBtn) {
            homeBtn.addEventListener("click", jumpToStart);
        }
        if (endBtn) {
            endBtn.addEventListener("click", jumpToEnd);
        }

        if (driftPrevBtn) {
            driftPrevBtn.addEventListener("click", function () {
                jumpDrift(-1);
            });
        }
        if (driftNextBtn) {
            driftNextBtn.addEventListener("click", function () {
                jumpDrift(1);
            });
        }

        if (speedContainer) {
            speedContainer.addEventListener("click", function (evt) {
                var target = evt.target;
                while (target && target !== speedContainer &&
                        !target.hasAttribute("data-speed")) {
                    target = target.parentNode;
                }
                if (target && target.hasAttribute && target.hasAttribute("data-speed")) {
                    var speed = parseFloat(target.getAttribute("data-speed"));
                    if (!isNaN(speed)) {
                        store.setSpeed(speed);
                    }
                }
            });
        }

        store.subscribe(function (changedKeys, state) {
            if (changedKeys.indexOf("t") !== -1) {
                reflectSeek(state.t);
            }
            if (changedKeys.indexOf("playing") !== -1) {
                handlePlayingChange(state.playing);
            }
            if (changedKeys.indexOf("speed") !== -1) {
                reflectSpeed(state.speed);
            }
        });

        // Sync the DOM to the store's actual current values right away
        // (the static markup's placeholders are zeros/defaults).
        reflectSeek(store.t);
        reflectSpeed(store.speed);
        updatePlayGlyph(store.playing);

        window.addEventListener("keydown", onKeyDown);
    }

    CTPS._inits.push(init);
})();
