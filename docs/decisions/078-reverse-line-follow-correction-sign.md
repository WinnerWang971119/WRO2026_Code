# ADR-078: Reverse line-follow negates the P+D correction sign

**Status:** Implemented
**Date decided:** 2026-07

## Rule

`LineFollower::update()` negates its P+D correction when `_reverse` is set:
`corr = LINE_KP_REAR*(err - offset) + LINE_KD_REAR*dErr`, then `corr = -corr`. Only the
correction polarity flips; cruise is negated separately (backward travel) and `getError()` /
`getRawError()` stay the raw chassis-frame centroid. The negation covers the P and D terms
together, so the loop's equilibrium (`err == offset`) is unchanged — ADR-069's travel-relative
offset and ADR-070's rear gains compose on top without modification.

## Why

Reverse follow reads the REAR board (`_activeBoard == 1`), which sits BEHIND the axle; the
front board sits ahead of it. For one body rotation, a point ahead of the axle and a point
behind it sweep in OPPOSITE lateral directions. The P+D law is tuned so the *front* board
chases the line, so the same correction applied to the *rear* board steers it AWAY from the
line — positive feedback. On the bench this showed up as reverse follow hunting and never
converging (oscillating off the line, then lost-line STOP). Negating the correction restores
negative feedback. This is the polarity item ADR-069 flagged for bench confirmation ("confirm
LEFT/RIGHT polarity on the front AND rear line boards"). The earlier claim — that
differential-drive symmetry means the P+D mix needs no change in reverse — was written for a
single FRONT-mounted tracer and is true there; it does not hold once the *leading* sensor
moves to the far side of the axle.

## How to apply

- The flip lives in the controller (`LineFollower::update()`, the online non-junction P+D
  block), gated on `_reverse` — NOT in `WROLineArray::getError()`. Keeping the sensor a pure
  chassis-frame reporter avoids a second sign interaction with the junction LEFT/RIGHT
  classification `Intersection` consumes and with ADR-069's PathTracker offset composition.
- Negate the WHOLE `corr` (after computing P+D, before the `LINE_MAX_CORR` clamp). Negating P
  and D together preserves the settling point `err == offset`, so no offset double-flip.
- If a future chassis puts the reverse-leading sensor AHEAD of the axle (or the rear board is
  physically wired with mirrored channel order), this flip must be revisited — it assumes the
  reverse-active board trails the axle in the direction of travel.

## Trigger to revisit

- Bench measurement shows reverse follow still diverges after this flip → the physical rear
  board channel order is mirrored from the assumed convention; move the sign to the sensor
  layer instead and re-derive the ADR-069 offset composition.
- A 1-board reverse-follow configuration is ever used (rear-less chassis tracking the front
  board while driving backward) — the front board then trails, which is a different stability
  case than the 2-board rear-leads case this ADR covers.

## See also

- [ADR-069 — LineFollower lateral offset-follow](./069-line-follow-lateral-offset.md) — the
  travel-relative offset sign flip PathTracker applies; this ADR preserves its equilibrium.
- [ADR-070 — LineFollower front/rear gains](./070-line-follower-front-rear-gains.md) — the
  per-board gain selection this sign flip composes with.
- [ADR-024 — Bidirectional line-following](./024-bidirectional-line-following.md) — the
  front/rear active-board routing that makes reverse follow read the rear board.
- `WRO2026/src/drive/LineFollower.cpp` — `update()` corr-negation block.
