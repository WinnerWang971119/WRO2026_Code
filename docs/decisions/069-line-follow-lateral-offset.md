# ADR-069: LineFollower lateral offset-follow — setpoint-bias LEFT/RIGHT, directional and travel-relative

**Status:** Implemented
**Date decided:** 2026-07 (issue #156)

## Rule

A LINE edge's `mode` may be authored `LEFT`/`RIGHT` (alongside `MIDDLE`) to bias
`LineFollower`'s P+D loop off-center, shifting the chassis laterally while it keeps
tracking the line. The bias is a signed offset on the P term only —
`corr = LINE_KP*(getError() - offset) + LINE_KD*(getError() - _lastErr)`, with `_lastErr`
staying the **raw** centroid — so `getRawError()` keeps its existing contract and a
mid-follow mode change injects no derivative kick. `mode` is **directional** for LINE
edges (LEFT/RIGHT does not mirror to the reverse-direction entry; BLIND `mode` stays
bidirectional) and **travel-relative**: `PathTracker` negates the resolved offset on a
rear-board (reverse) hop, so LEFT always means left of the direction of travel. `DEFAULT`
resolves to `MIDDLE` (offset 0); there is no `PIPELINE_CONFIG` follow-offset default.
`EdgeInfo` grows an 8th field, `line_follow_mode_override: LineFollowMode`, and every
generated + hand-authored `EDGES` cell now goes through one uniform `constexpr
makeEdge(...)` factory.

## Why

ADR-064 orthogonalized the per-edge Speed and Mode axes but left LINE Mode
surface-only — `MIDDLE` validated and emitted nothing — pending this feature. Issue #156
asks for a line-following edge biased to one side, to clear an obstacle or line up a
manipulator without leaving the line. Setpoint-bias reuses the existing 8-channel
centroid: on a thin dark line an "edge" coincides with the centroid within a channel, so a
literal edge-follower buys nothing, and dead-reckoning a lateral offset is rejected by
ADR-001 and ignores the line entirely. Biasing the P term only (not the derivative or the
raw-error telemetry) keeps the change local to the setpoint the loop already drives to
zero, so junction-freeze, the distance gate, and lost-line handling are untouched.

## How to apply

- `Config::LINE_FOLLOW_OFFSET_ERROR` (`ConfigLine.h`, seeded `1.5f`, centroid-index
  units, LEFT=+/RIGHT=-) with a `static_assert(0.0f <= x <= 2.5f)` in `ConfigChecks.h` —
  the cap sits above the seed so a bench retune upward doesn't trip it, and still keeps
  `LINE_KP*offset` under `LINE_MAX_CORR`.
- `LineFollower::setLineFollowOffset(float)` is a nav-agnostic float builder knob (no
  `Routes::` dependency in the drive layer). Lifecycle follows the `setOvershootMode`/
  `setBrakeOnGate` (ADR-023) reset-on-disable pattern, not `setPreset`/`setReverse`: cold
  `setEnabled(true)` with no override is 0, a re-arm on an already-enabled follower
  preserves the current offset, `setEnabled(false)` resets it to 0.
- `PathTracker::_resolveLineFollowMode(edge)` maps `DEFAULT`→`MIDDLE`; `WROMath::
  lineFollowModeToSetpoint(mode)` converts to a signed float; `PathTracker` composes
  `sign = decision.reverse ? -1 : +1` and chains `.setLineFollowOffset(sign * setpoint)`
  into `LINE_FOLLOW_INTERSECTION` and `PROJECTION_FOLLOW`. Not added to
  `EdgeDispatchQuery`/`Decision` — that 7-field truth table is untouched.
- `validator.py` accepts `MIDDLE|LEFT|RIGHT` on LINE (drops the issue-#156 reject);
  `graph_builder.py` treats LINE `mode` as directional and BLIND `mode` as bidirectional;
  `build_routes.py` warns (does not fail) when a used reverse traversal of a biased LINE
  edge has no companion authored entry.
- `makeEdge(type, distance_cm, heading_deg, speed_override, blind_mode_override,
  direction_override, stop_at_line_override, line_follow_mode_override)` is emitted into
  `routes.h` and every shadow-routes fixture header, and used for every `EDGES` cell
  (including `NONE`) and by `FieldMap::noEdge()`. Two mechanical guards keep the ~30 KB
  `EDGES` array in flash `.rodata`: an emitted `constexpr`-evaluability canary
  `static_assert`, and a CI check on `arduino-cli compile`'s "Global variables use N
  bytes of dynamic memory" figure against a pinned ceiling.
- Parity bumps to schema v3 in lockstep across six surfaces (both C++ emitters, the C++
  header schema, both C++ version-stamp sites, and the Python fields + version):
  `line_follow`/`projection_follow` records gain `follow_mode` (the resolved member-name
  string, pre-reverse-sign — the mode, not the signed offset) and rename `line_mode` →
  `speed` in the same bump.
- CTPS renders the bias as a **visualization-only** lateral chassis nudge on the biased
  leg (the ADR-054 pivot-wheel-arc pattern), mapped through a named CTPS-layer constant
  `LINE_FOLLOW_OFFSET_DISPLAY_CM`; the parity-locked `LocalizationSim` pose is never
  touched.
- **Accepted, unmodeled approximation:** a biased arrival lands the axle roughly 1–1.5 cm
  laterally off the node versus what `Localization`/the sim assume. This sits inside
  junction-arrival tolerance and is left uncompensated in both — the project has a scar
  from adding per-junction sim offsets before (see ADR-054's own trigger list).
- **Bench items, pre-competition:** confirm LEFT/RIGHT polarity on the front AND rear
  line boards; check `getJunctionType()` classification and `isLineLost()` still behave
  correctly at full offset (the crossbar channel pattern shifts under the sensor); tune
  `LINE_FOLLOW_OFFSET_ERROR` on the actual mat.

## Trigger to revisit

- A mission needs a numeric per-edge offset *magnitude*, not just the LEFT/RIGHT sign —
  today one global `Config` constant sets the magnitude for every biased edge; a
  per-edge value is a new field and a new ADR.
- A `goTo(...)` call-site override is requested (mirroring a NavTypes call-site override
  elsewhere) — today the mode is edge-authored only, like `speed`/`blind_mode`.
- The bench-observed axle offset turns out to exceed junction-arrival tolerance on the
  real field, forcing the approximation above to be compensated after all.
- `makeEdge` grows a 9th field — the factory signature, the canary, and all three
  shadow-routes fixtures change together.

## See also

- [ADR-064 — Per-edge Speed/Mode orthogonalization](./064-per-edge-mode-speed-orthogonalization.md)
  — its "LINE Mode is surface-only this PR" clause (`MIDDLE`-only, no field) is
  **superseded by this ADR**; the rest of ADR-064 (the Speed axis, the `STOP_AT_LINE`
  fold) stands unchanged.
- [ADR-054 — CTPS pivot wheel-arc](./054-ctps-pivot-wheel-arc.md) — the visualization-only
  chassis-nudge pattern this ADR's CTPS render reuses (dispatch/pose stay parity-safe).
- [ADR-023 — Pass-through collinear intersections + overshoot-mode probe](./023-pass-through-collinear-intersections.md)
  — the `setOvershootMode`/`setBrakeOnGate` reset-on-disable builder lifecycle this ADR's
  offset knob follows.
- [ADR-050 — PathTracker sim↔firmware FSM parity](./050-pathtracker-sim-parity.md) — the
  trace-diff job this ADR's schema v3 bump must keep green.
- [ADR-001 — No wall-follow, no dead-reckoning](./001-no-wall-follow-no-dead-reckon.md) —
  why the bias is a setpoint shift on the existing centroid, not a dead-reckoned offset.
- `docs/plans/2026-07-17-issue-156-lateral-offset-line-follow.md` — originating plan
  (Decisions section).
- Issue #156 (closed by this).
