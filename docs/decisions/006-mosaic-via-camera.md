# ADR-006: Read the 4×3 mosaic with a camera at a line-anchored stop

**Status:** Implemented (Refined by ADR-037)
**Date decided:** 2026-05 (Phase 0)

## Rule

The mosaic pattern is read by camera at a line-anchored full stop. The HUE12
camera reads **12 cells** arranged in a **4×3 grid** (4 columns × 3 rows,
confirmed from `WRO2026_HighSchool_Topic_Drawer.html` game rules, 2026-06-13).
Each cell returns a classified color; the complete latch is captured in one
`readMosaic()` call. The camera-to-cell geometry is fixed by construction and by
the line-anchored stop position — not by odometry.

*Slot vs. cell:* "slot" refers to a physical location in the mosaic fixture;
"cell" refers to the logical index (0–11) used in the `Vision` API. Each slot
corresponds to exactly one cell.

## Why

Without reading the mosaic, the mosaic placement task is unreachable. Pure
odometry won't give millimeter-accurate camera positioning; a few millimeters of
drift puts the sample point on the wrong cell. Line + intersection anchoring is
the only reliable method.

## How to apply

- The camera is the HUE12 module communicating over `Serial1`; see `Vision.h`.
- Lighting baselines for color classification must be recalibrated **on-site**
  before each run (workshop ≠ venue).
- Sensor mount geometry must be locked before software calibration — see Phase 3
  entry criteria.
- The mosaic contains 12 cells, not 9. Any field encoding, task body, or
  benchmark that referenced 9 cells or a 3×3 grid is stale and must be updated.

## Trigger to revisit

If line-anchoring at the mosaic frame proves unreliable (e.g., the approach line
is too close to a busy zone), a fall-back to wheel-encoder-based fine positioning
would need its own ADR. Frame-contract or baud-rate assumptions are revisited via
ADR-037.

## See also

- [ADR-037](./037-hue12-vision-capture-once.md) — capture-once latch, bounded-resync parser, on-device hue classification (refines this ADR)
- [ADR-021](./021-camera-as-sole-color-sensor.md) — camera as sole color sensor
- [Roadmap.md](../../Roadmap.md) — Phase 3 perception
