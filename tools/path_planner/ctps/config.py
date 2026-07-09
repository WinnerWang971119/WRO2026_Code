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
#
# The former ``SIM_DRIVE_CM_S`` / ``SIM_TURN_DEG_S`` flat-timing constants were
# retired in issue #130 (T10): drive/turn primitive display durations are now
# derived from the firmware T-shape motion profile (``ctps/profile.py``),
# scaled onto the timeline by the single visual constant
# :data:`SCALE_CM_S_PER_PWM` below. There is no longer a constant-velocity
# playback speed.

#: Narration dwell time applied to any ``manip`` command in the playback
#: (seconds).  The Manipulator stub has no distance or angle telemetry so
#: a fixed nominal duration is used instead.
MANIP_NOMINAL_S: float = 0.5

# ---------------------------------------------------------------------------
# Firmware-derived motion profile — visual scale (issue #130)
# ---------------------------------------------------------------------------
#
# The playback motion profile (``ctps/profile.py``) samples dense keyframes
# from the SAME T-shape ramp math the firmware runs (``TShapeProfilePolicy``),
# driven off the real ``ConfigDrive.h`` constants. That gives a PWM-domain
# speed at every point along a move; converting PWM to an on-screen cm/s rate
# needs exactly one visual scale, below. There is deliberately no second
# scale for turns: angular rate is derived from this same constant plus the
# chassis track width (``planner.parse_track_cm_from_config()``, itself read
# from ``Config::TRACK_MM``) inside ``profile.py`` — see
# ``resolve_turn_profile``.

#: CTPS-only visual scale: on-screen drive speed (cm/s) per commanded PWM
#: unit. This is the SINGLE tunable that maps the firmware's PWM-domain
#: T-shape profile onto the playback timeline; retuning firmware ramp/cruise
#: constants changes playback duration with no CTPS-side edit needed.
#:
#: Chosen so a move at the default cruise preset (``ConfigDrive::SPEED_NORMAL
#: = 85`` PWM) renders at the historical ~30 cm/s playback feel (the flat
#: ``SIM_DRIVE_CM_S`` this replaced):
#:
#:   30 cm/s at cruise PWM 85 -> 30 / 85 = 0.3529411...
#:
#: Not a physical calibration (no PWM-to-cm/s measurement) — purely a display
#: constant preserving today's playback pace. Do NOT use to estimate real
#: robot speed.
SCALE_CM_S_PER_PWM: float = 30.0 / 85.0

# ---------------------------------------------------------------------------
# Firmware-derived motion profile — keyframe sampling knobs (issue #130)
# ---------------------------------------------------------------------------
#
# `profile.py` (T5) emits dense-but-bounded keyframes along each ramp phase;
# `track_builder.py` (T10) consumes the resulting phase-object profile when
# densifying drive/line/turn primitives. These two knobs bound that
# densification: a maximum spatial gap between interior ramp samples, and a
# hard ceiling on how many keyframes a single primitive may emit (protects
# the player from a pathological/degenerate profile flooding the timeline).

#: Maximum distance (cm) between consecutive interior keyframes sampled along
#: a ramp (accel/decel) phase. Smaller values track curvature in the T-shape
#: speed-vs-distance curve more faithfully at the cost of more keyframes;
#: cruise-phase and floor-crawl stretches are still sampled sparsely (only at
#: phase boundaries) since PWM is constant there and a dense sample adds no
#: visual information.
RAMP_SAMPLE_STEP_CM: float = 2.0

#: Hard ceiling on keyframes emitted per drive/turn primitive by
#: `sample_drive` / `sample_turn`. Bounds pathological cases (e.g. a very
#: long ramp at a tiny `RAMP_SAMPLE_STEP_CM`) so a single primitive can never
#: flood the player timeline; sampling degrades to coarser spacing rather
#: than exceeding this count.
KEYFRAME_COUNT_CEILING: int = 200

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
# Robot physical frame (rendering)
# ---------------------------------------------------------------------------

#: The competition robot's physical chassis footprint, drawn as the robot
#: marker in the CTPS player and the layout editor (a square of this side, in
#: centimetres).
#:
#: This is the TRUE physical frame and is deliberately DISTINCT from the
#: planner's ``field_data.ROBOT_RADIUS_CM`` C-space inflation radius: the frame
#: is what the operator eyeballs against the field, while the C-space radius is
#: a planning artifact that over-represents the footprint (a 15 cm radius would
#: otherwise draw a 30 cm box). Injected into the render ``field`` block by
#: :func:`tools.path_planner.ctps.track_builder.build_field_block` and read by
#: ``assets/renderer.js`` and ``assets/editor_core.js`` as ``robot_frame_cm``.
ROBOT_FRAME_CM: float = 25.0

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
