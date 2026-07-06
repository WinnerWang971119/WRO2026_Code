/* CTPS — Layout Editor: core (view transform + field backdrop + tab state).
 *
 * PLACEHOLDER (object/cargo plan, T7 -- issue #57). Inert stub reserving this
 * asset's slot in layout_editor_html._ASSET_JS -- inlined immediately AFTER
 * objects_draw.js (whose window.CTPS_OBJ it will reuse) so the editor emitter's
 * asset pipeline + self-containment lint + no-inline-style checks exercise the
 * final file layout now, ahead of T8 landing the real implementation:
 *
 *   - a DUPLICATED `_computeView` (the ~40-line world->pixel transform; NOT
 *     extracted from renderer.js, per the plan's Contracts -> Projection),
 *   - the field-backdrop draw from `const FIELD`,
 *   - Field/Robot tab state, and
 *   - reuse of `window.CTPS_OBJ.drawObject` / `drawZones` against the editor's
 *     own computed `view`.
 *
 * Establishes the editor's OWN namespace (`window.CTPS_EDITOR`, distinct from
 * the player's `window.CTPS`) so the sibling editor modules attach to it; core
 * loads first, so this is the module that seeds it.
 *
 * Inert today: no DOM access, no drawing, no network. Vanilla ES5, no
 * frameworks, works offline. Never sets an inline element style (setProperty
 * on a tokens.css custom property is the only sanctioned styling channel).
 */

"use strict";

(function () {
    window.CTPS_EDITOR = window.CTPS_EDITOR || {};
})();
