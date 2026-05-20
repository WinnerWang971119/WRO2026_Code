# ADR-006: Read the 3×3 mosaic with a camera at a line-anchored stop

**Status:** Pending implementation (Phase 3)
**Date decided:** 2026-05 (Phase 0)

## Rule

The mosaic pattern is read by camera: the robot positions itself in front of the frame, captures one image, samples small pixel regions at the center of each of the 9 cells, averages the RGB values, and classifies each cell against pre-calibrated baselines for yellow, blue, green, and white.

The camera's position relative to the frame comes from **line-following navigation** — the robot approaches the frame along a known line and stops at a known intersection. The camera-to-cell geometry is fixed by construction, not by odometry.

## Why

Without reading the mosaic, the 120-point mosaic placement task is unreachable. Pure odometry won't give millimeter-accurate camera positioning; a few millimeters of drift puts the sample points on the wrong cells. Line + intersection anchoring is the only thing reliable enough.

## How to apply

- Camera selection (HuskyLens vs raw + on-device pixel classify) is still open — see [Roadmap.md](../../Roadmap.md) Open Decisions Log.
- Lighting baselines for color classification must be recalibrated **on-site** before each run (workshop ≠ venue).
- Sensor mount geometry must be locked before software calibration — see Phase 3 entry criteria.

## Trigger to revisit

Camera selection closes (will instantiate `CameraMosaic` implementation). If line-anchoring at the mosaic frame proves unreliable (e.g., the approach line is too close to a busy zone), a fall-back to wheel-encoder-based fine positioning would need its own ADR.

## See also

- [Roadmap.md](../../Roadmap.md) — Phase 3 perception
