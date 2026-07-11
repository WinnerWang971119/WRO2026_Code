/* CTPS — Clear Trajectory Prediction Simulator: randomized-vision stub.
 *
 * Placeholder for the CTPS randomized-vision feature (mosaic + robot-attached
 * vision cone + read-mosaic HUD prediction — issue tracked in
 * docs/plans/2026-07-11-ctps-randomized-vision.md). This file exists ONLY so
 * that every task which unconditionally inlines this asset (the player T10,
 * the layout editor T13) finds a real file on disk — the upload leg needs
 * `vision.js` present on every page regardless of whether a mosaic was rolled
 * (ADR-056 unconditional-assets precedent).
 *
 * T0 (this file, now): a syntactically-valid no-op. Registers no layers, runs
 * no work, reads no store/DOM.
 * T11 (later): fleshes this out with `visibleCells`, `readMosaicWouldSucceed`,
 * `mosaicCellCenters`, `drawVisionCone`/`drawMosaicCells`, the per-frame
 * cone/mosaic layer registrations, and the `#ctps-vision` HUD wiring.
 *
 * Vanilla ES5 — var/function only, works offline. No DOM writes, no network.
 */

"use strict";

(function () {
    // Empty namespace for T11 to extend. Declared now so any early reference
    // (e.g. from the editor bootstrap) finds a defined object rather than
    // undefined.
    window.CTPS_VISION = window.CTPS_VISION || {};
})();
