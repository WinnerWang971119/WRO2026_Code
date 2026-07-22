# ADR-081: CTPS declare_position snaps the rendered chassis to the node

**Status:** Implemented
**Date decided:** 2026-07 (Phase 5)

## Rule
In the CTPS player, a `declare_position` op teleports the rendered chassis to the
declared node's coordinates (position only — heading is left unchanged) and the
following `goTo` departs from that node. This reverses the ADR-058 / T11
"belief-only, no chassis mutation, no keyframe" rendering of the same op. An
unknown-node declare still no-ops (no marker, no snap).

## Why
`declarePosition` is an operator **ground-truth re-anchor**: it asserts the robot
IS at the named node (e.g. after a wall-square or a known landmark) and zeroes the
accumulated drift. The original belief-only rendering reset only the sim's internal
belief, leaving the drawn robot on its drifted dead-reckoned pose — so the operator
could not SEE the re-anchor, and the subsequent `goTo` was routed from the declared
node but rendered starting from the wrong (drifted) place. Snapping the chassis to
the node makes the predicted trajectory pick up from where the operator declared the
robot to be, which is what the re-anchor means and what the operator expects to see.

## How to apply
`tools/path_planner/ctps/track_builder.py`, the `declare_position` branch: after the
sim belief reset, close any open collinear run, teleport the shared `VirtualChassis`
to `field_data.NODES[node]` (x/y only — a declare leaves heading untouched, mirroring
the firmware leaving `_yawOffsetDeg` alone), and emit an instantaneous snap keyframe
sharing the previous keyframe's timestamp (the player's `poseAt` zero-length-bracket
guard renders it as a hard jump, not a slide). The chassis move lives in the RENDER
layer only: the sim's `LocalizationSim.declare_position` stays a pure belief reset, so
the parity harness and `runtime.py` goldens are unaffected. The firmware is untouched —
`declarePosition` remains a belief correction on the real robot; only the CTPS
prediction is re-anchored.

## Trigger to revisit
If a mission needs to visualize the PHYSICAL drift for diagnostics — i.e. show where
dead-reckoning *thought* the robot was versus the declared ground truth — a dual
rendering (a ghost at the drifted pose plus the snap) would supersede the hard snap.

## See also
- ADR-058 (CTPS firmware motion profile; introduced `declarePosition` as a first-class
  CTPS op with belief-only rendering — **amended** here).
- `tools/path_planner/ctps/track_builder.py`, `tools/path_planner/sim/runtime.py`
  (`LocalizationSim.declare_position`), issue #130.
