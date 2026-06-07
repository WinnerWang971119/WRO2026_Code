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
#: Rationale: after a CLEAN ``goto`` the chassis lands ON the node
#: (LINE_OVERSHOOT advances the axle onto it) so drift ≈ 0.  A raw Drive
#: move between hops injects a real offset that ``goto`` does not correct.
#: 10 cm sits above float/rounding noise and benign dead-reckon residual
#: while still catching a raw-move desync.  Drift is measured AFTER each
#: ``goto``'s final hop, comparing chassis pose to NODES[current_node].
DRIFT_THRESHOLD_CM: float = 10.0
