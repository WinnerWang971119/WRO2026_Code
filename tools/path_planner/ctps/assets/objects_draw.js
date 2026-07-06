/* CTPS — Clear Trajectory Prediction Simulator: field-object + zone canvas draws.
 *
 * PLACEHOLDER (object/cargo plan, T4 -- issue #57). This file's only job today
 * is to establish the shared `window.CTPS_OBJ` namespace so:
 *
 *   - `player_html._ASSET_JS` can register this asset (and the sibling
 *     `cargo.js`, which references `CTPS_OBJ` at IIFE time) ahead of T5
 *     landing the real implementation, exercising the emitter's asset
 *     pipeline + self-containment lint against the final file layout now.
 *   - T5 fills in the transform-agnostic canvas draws:
 *     `CTPS_OBJ.drawObject(ctx, view, obj, project, opts)` and
 *     `CTPS_OBJ.drawZones(ctx, view, pose, robotCfg, project)`, both driven
 *     through a `project(view, x, y)` world->pixel helper (see the plan's
 *     Contracts -> Player-side seams -> Projection). `renderer.js` is NOT
 *     touched -- this module owns its own projection formula.
 *   - T8 (the layout editor) reuses the same `CTPS_OBJ.drawObject`/
 *     `drawZones` against its own computed `view`.
 *
 * Inert today: no drawing, no DOM access, no store access. Concatenated into
 * its own <script> element, immediately BEFORE `cargo.js` and after every
 * other player module. Vanilla ES5, no frameworks, no network, works offline.
 */

"use strict";

(function () {
    window.CTPS_OBJ = window.CTPS_OBJ || {};
})();
