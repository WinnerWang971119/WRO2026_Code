"""Visual timing and threshold constants for the CTPS playback renderer.

All values here are **visual-only** — they govern how long each command
appears to take on screen during playback.  They are NOT physical-duration
predictions and do NOT reflect real robot speed.  Do not use these to
estimate actual competition run time.

Also holds the fixed manipulator cargo-model player constants (object/cargo
plan, issue #57) — magazine geometry and rear-box capacity are NOT authored
per-layout (:mod:`tools.path_planner.ctps.layout`); they are single-sourced
here and mirrored as JS constants in ``assets/cargo.js``.
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

# ---------------------------------------------------------------------------
# Pivot-turn arc rendering
# ---------------------------------------------------------------------------

#: Degrees of rotation per sub-keyframe when the pivot-turn arc (a single-wheel
#: pivot rendered as a rigid rotation about the planted/braked wheel) is
#: expanded into dense keyframes for playback.
#:
#: Rationale: at the pivot radius used on this chassis (``track_cm / 2`` ≈ 9 cm),
#: a 10° sub-step has a sagitta (chord-to-arc deviation) of only
#: ``R · (1 − cos(step / 2))`` ≈ 0.14 px at typical playback scale — sub-pixel —
#: so the arc reads as smooth without emitting an excessive number of keyframes.
ARC_STEP_DEG: float = 10.0

# ---------------------------------------------------------------------------
# Manipulator cargo-model player constants (object/cargo plan, issue #57)
# ---------------------------------------------------------------------------
#
# Fixed player constants, NOT authored per-layout: `layout.json` (T2) carries
# only field objects and robot zone geometry, never magazine/rear-box
# capacity, so a layout author can never desync the player's cargo-store
# shape from what the panel renders. `ctps/layout.py`'s `validate()` reads
# `MAGAZINE_LANES` to check `robot.drop_points` has exactly one entry per
# lane; the resolver/panel (T5/T6) read all three.

#: Number of vertical magazine lanes the manipulator files sorted tiles into
#: (`sortTo(n)` for `n` in `[0, MAGAZINE_LANES)`). Matches the Robot tab's 4
#: drop points — one per lane — and the player/editor magazine grid width.
MAGAZINE_LANES: int = 4

#: Maximum tiles stacked in a single magazine lane (front-to-back depth; the
#: front tile sits at the bottom, the dump side).
MAGAZINE_DEPTH: int = 3

#: Rear-box (Inventory) capacity — how many Cargo objects a single
#: `boxClose` can hold at once.
REAR_BOX_CAP: int = 1
