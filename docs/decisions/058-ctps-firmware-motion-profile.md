# ADR-058: CTPS playback renders the firmware's T-shape motion profile

**Status:** Implemented
**Date decided:** 2026-07 (phase, issue #130)

## Rule

CTPS playback speed is **derived from `ConfigDrive.h`**, not a fixed cartoon
rate. A single Python "phase-object kernel" (`tools/path_planner/ctps/profile.py`)
computes each primitive's accel→cruise→decel shape from **closed-form phase
durations** (never forward-Euler `dt = ds/(k·pwm)` as the render clock — that
diverges at `endSpeed=0`) and models the deadband floor explicitly:
`v(s) = k·max(pwm(s), PWM_MIN_MAGNITUDE)`, so a stop is a decel ramp, a
floor-crawl, then a terminal brake keyframe — never an asymptotic crawl. The
kernel is pinned to the firmware by extracting the pure distance-domain ramp
math out of `Drive.cpp` into a header-only `TShapeProfilePolicy.h`; a host TU
dumps golden vectors and `profile.py` must reproduce them bit-for-bit
(float32-emulated) under CI. `profile.py` emits **dense, non-uniformly timed
keyframes** — display-only, sampled dense near ramps and sparse on cruise —
so `player.js`/`renderer.js` need no change; each primitive's last sub-keyframe
byte-equals the parity-locked sim's snapshot pose. Raw off-node drives route
through `PathTrackerSim.operator_move` so the following `goto` renders the
firmware's actual return-to-node maneuver instead of a CTPS-only drift model.
`Mission.declarePosition(name)` is a first-class recorded op
(`declare_position`), distinct from return-to-node, applied via
`LocalizationSim.declare_position` (node change + offset clear, heading
calibration preserved). The existing `drift` timeline lane is repurposed
(same lane id, relabelled "LOCALIZATION") to carry drift, return-to-node, and
declarePosition markers together. Turns and pivots render at the firmware's
**actual snapped PWM** — with current Config both turn stages deadband-snap
to `|PWM|=50`, so default turns render constant-rate; per-turn `setPower`/
`setPreset` override capture is deferred.

## Why

The prior constant-velocity playback (ADR-033) was chosen for smooth scrubbing
with no fidelity claim. Once the firmware's T-shape profile (ADR-014) is the
thing under test, a flat-rate render hides the exact behavior — speed-up,
slow-down, and stop shape — that the profile exists to produce, and gives no
signal when a Config retune changes real robot timing. Forward-Euler
integration was rejected because it is step-size sensitive and mathematically
diverges as commanded PWM approaches zero, which is exactly the terminal
behavior of every stop. Extracting the ramp math into a header-only policy
with a golden-vector pytest tether means the firmware itself stays the source
of truth: a retune regenerates the goldens, and a silent Python/firmware
desync fails CI instead of silently rendering the old shape forever.

## How to apply

- Profile math lives only in `ctps/profile.py`; never port ramp/duration math
  into `player.js` or `renderer.js`.
- Any change to `Drive.cpp`'s distance-domain ramp/triangle/half-cap or
  speed-at-fraction math must land in `TShapeProfilePolicy.h` first, then
  regenerate `tests/fixtures/tshape_profile_goldens.json` via the host TU —
  the ctest target fails loud on a stale committed fixture.
- Raw `drive` calls that leave a node must go through `operator_move`
  end-to-end (never a bespoke CTPS drift nudge) so the next `goto`'s return
  prologue is physically accurate.
- `declarePosition` recording sits on `Mission.cpp`'s success path only
  (after the busy/unknown-node guards), alongside the real
  `Localization.declarePosition` call.
- New localization-lane event kinds are additive to `{drift_warn,
  return_to_node, declare_position}`; both the Python emitter and the JS
  renderer must agree on the set.

## Trigger to revisit

- Turn/pivot per-command speed overrides (`setPower`/`setPreset` on
  `turnDeg`/`turnToYaw`) need capture — reopen the deferred-turn-override
  decision and extend the recorder + `resolve_turn_profile`.
- Wheel slip, battery sag, or IMU noise fidelity is requested — this profile
  is kinematic-only by design (see ADR-033's stated non-goals).
- The LRS (#23) supplies measured termination outcomes for `stopAtLine`/
  `wallStop`/`lineFollowDist`, at which point the nominal-`cm` host estimate
  those primitives still use should be revisited.

## See also

- [ADR-033 — CTPS ingests a derived mission trace recorded from the real C++](./033-ctps-derived-mission-trace.md) — playback fidelity clause amended by this ADR.
- [ADR-014 — T-shape trapezoidal velocity profile, acceleration-based, opt-in](./014-tshape-trapezoidal-velocity-profile.md) — the firmware profile this ADR renders.
- [ADR-051 — CTPS mock-builder surface kept a superset of the real builders](./051-ctps-mock-conformance-guard.md) — recorder extension for per-command speed knobs + `recordDeclarePosition`.
- [ADR-054 — CTPS pivot wheel-arc](./054-ctps-pivot-wheel-arc.md) — the pivot arc overlay this ADR re-times but does not restructure.
- [ADR-030 — Host-compile harness links the real production symbols, CI-enforced](./030-harness-links-real-sources-ci-enforced.md) — the golden-vector dump TU's CI enforcement pattern.
