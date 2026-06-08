"""Visual timing and threshold constants for the CTPS playback renderer.

All values here are **visual-only** — they govern how long each command
appears to take on screen during playback.  They are NOT physical-duration
predictions and do NOT reflect real robot speed.  Do not use these to
estimate actual competition run time.
"""

# ---------------------------------------------------------------------------
# Playback speed constants
# ---------------------------------------------------------------------------

#: Simulated drive speed used to compute how long a drive/line-follow
#: primitive occupies on the playback timeline (cm per second).
SIM_DRIVE_CM_S: float = 30.0

#: Simulated turn speed used to compute turn-primitive display durations
#: (degrees per second).
SIM_TURN_DEG_S: float = 180.0

#: Narration dwell time applied to any ``manip`` command in the playback
#: (seconds).  The Manipulator stub has no distance or angle telemetry so
#: a fixed nominal duration is used instead.
MANIP_NOMINAL_S: float = 0.5

# ---------------------------------------------------------------------------
# Drift detection threshold
# ---------------------------------------------------------------------------

#: Maximum chassis-vs-node divergence (in centimetres) before the playback
#: engine emits a drift warning.
#:
#: Rationale: drift is measured AFTER each ``goto``'s final hop, comparing the
#: chassis pose to NODES[current_node].  A CLEAN ``goto`` lands the wheel axle ON
#: the node regardless of final-hop type — a LINE edge's LINE_OVERSHOOT step rolls
#: the axle exactly onto the node (net advance == edge length, matching the
#: firmware re-anchor), and blind/projection/stop-at-line hops land on the node by
#: construction — so drift ≈ 0.  A raw Drive move between hops injects an offset
#: that ``goto`` does not correct.  10 cm clears float/rounding noise and a benign
#: dead-reckon residual while still catching a raw-move desync; revisit once the
#: live field lands.
DRIFT_THRESHOLD_CM: float = 10.0
