# Issue #22 — LineFollower PID Freeze During Any Intersection

**Goal:** Suppress the centroid-skew wobble that occurs when the chassis crosses any junction (LEFT, RIGHT, or T_OR_CROSS) by freezing the P+D mix and commanding symmetric cruise for the duration of the junction dwell.

**Approach:** Add a single freeze gate inside `LineFollowerClass::update()` that reads `_array->getJunctionType()` once per tick on the raw per-tick value (not the debounced edge). When the verdict is anything other than NONE or UNKNOWN, skip the err/dErr/corr math and command `Drive.setSpeed(cruise, cruise)` (the Drive layer applies motor-bias calibration automatically). On the first non-junction tick after a freeze, reset `_lastErr` to 0 to prevent a stale-D-term kick. Trapezoidal ramps and the encoder distance gate continue ticking unaffected. Broadened from issue #22's original T_OR_CROSS-only scope per Diego's request — the centroid-skew defect affects LEFT/RIGHT just as much (and the issue body itself flagged this).

## Scope

- **In scope:**
  - Freeze the P+D mix in `LineFollower::update()` whenever `_array->getJunctionType()` returns LEFT, RIGHT, or T_OR_CROSS.
  - Hold both wheels at the resolved cruise PWM (sign-flipped for reverse) during freeze ticks; motor-bias calibration applied automatically by `Drive.setSpeed`.
  - Reset `_lastErr = 0` on the first non-junction tick after a freeze (D-term kick prevention).
  - Distance-gate evaluation and trapezoidal ramps continue ticking during freeze.
  - Add `LineFollowerClass::isPidFrozen() const` accessor for Serial debug / future telemetry.
  - Update the file/class doxygen on `update()` to document the freeze branch.

- **Out of scope:**
  - Tick-scoped caching of `getJunctionType()` on `WROLineArrayClass` — orthogonal optimization noted in PR #20 review; tracked separately. (When `LINE_ARRAY_TICK_CACHE_ENABLED == true`, the call is already free; when `false`, the extra ~9 ms read is accepted for this change.)
  - EventTrace integration — explicitly skipped to preserve OLED ring capacity for existing events.
  - Hysteresis / debounce on freeze enter or exit — instant both ways per design.
  - IMU yaw-hold during freeze — not chosen; reconsidered only if symmetric cruise drifts visibly off-line in field tests.
  - Changes to `Intersection.update()`, `WROLineArrayClass`, `Config`, `EventTrace`, or the fluent builder API.

## Decisions

- **Freeze output = symmetric cruise + motor bias.** L = R = `cruise` (the same `cruise` variable already computed at LineFollower.cpp:163, post-ramp). `Drive.setSpeed(L, R)` applies the per-motor bias calibration automatically (DECISIONS.md "Strategy B"). — Matches issue's proposal and existing mem0 design rule; simplest and obviously correct.
- **Trigger scope: any non-NONE non-UNKNOWN classification.** LEFT, RIGHT, and T_OR_CROSS all freeze. — Broader than issue #22; the issue body itself notes LEFT/RIGHT have "the same and even bigger problem" with centroid skew.
- **Instant enter, instant exit.** Freeze on the first tick of a non-NONE reading; release on the first NONE. — Best wobble suppression; the wobble we're fixing starts on tick 1, so any debounce defeats the purpose.
- **Read source: direct `_array->getJunctionType()` call.** — Single line, no API churn. With `LINE_ARRAY_TICK_CACHE_ENABLED == true` it's free; with caching off it adds one I²C transaction per tick (~9 ms), accepted.
- **D-term reset on freeze EXIT only.** Set `_lastErr = 0` on the first non-frozen tick after a freeze. — Mirrors the lost-line branch pattern at LineFollower.cpp:281; PID isn't running during freeze so reset on ENTER is redundant.
- **Lost-line dispatch takes precedence over freeze.** Check `_array->isOnline()` first (unchanged), then the freeze gate. — Safety-critical path runs first; defensive ordering even though `getJunctionType()` returns NONE when no channel is on-line under current `WROLineArrayClass` semantics.
- **Structural placement: if/else around the P+D block, NOT an early return.** — So the distance-gate code at LineFollower.cpp:344-380 continues to run on freeze ticks.
- **No new Config constants.** — Existing `LINE_JUNCTION_T_MIN_ON` / `LEFT_HI` / `RIGHT_LO` already gate the classifier output, which is the only knob the freeze depends on.
- **No EventTrace integration.** — Preserves OLED ring capacity for existing events; `isPidFrozen()` accessor remains for Serial debug.
- **Freeze applies in reverse too.** — `cruise` is already sign-flipped at LineFollower.cpp:163-167; freeze just commands symmetric `cruise` so the same code path covers both directions.

## Acceptance Criteria

- [ ] When `_array->getJunctionType()` returns LEFT (1), RIGHT (2), or T_OR_CROSS (3), `LineFollower::update()` skips the err/dErr/corr/clamp/mix block and commands `Drive.setSpeed(cruise, cruise)` (deadband-snapped).
- [ ] When the classifier returns NONE (0) or UNKNOWN (4), normal P+D execution resumes from the next tick.
- [ ] On the first non-frozen tick after a frozen tick, `_lastErr` is reset to `0.0f` before the P+D math runs (no derivative kick).
- [ ] During freeze ticks, the encoder distance-gate evaluation (LineFollower.cpp:350-380) continues to run; if the gate trips during a freeze tick the controller self-disables and brakes normally.
- [ ] During freeze ticks, the trapezoidal ramp-up and ramp-down branches (LineFollower.cpp:169-275) continue to evolve `cruise` normally; the freeze uses whatever `cruise` value the ramp produced this tick.
- [ ] Freeze behaviour is identical in forward and reverse (cruise sign already handled at lines 163-167).
- [ ] `Intersection.update()` edge semantics are unchanged — `INTERSECTION_DEBOUNCE_TICKS` / `RELEASE_TICKS` still gate edge firing; freeze does not pre-empt or alter edge detection.
- [ ] Lost-line dispatch (`Config::LOST_LINE_ACTION`) still fires when `!_array->isOnline()` regardless of any concurrent junction classification.
- [ ] `LineFollowerClass::isPidFrozen()` returns `true` on freeze ticks, `false` otherwise. Read-only; does not mutate state.
- [ ] When the controller is disabled (`_enabled == false`), `isPidFrozen()` returns `false` and the freeze state is cleared (no leaked freeze flag into the next enable cycle).
- [ ] No changes to the fluent builder, `setEnabled` semantics, `setPreset`, `setReverse`, or any public method besides the new `isPidFrozen()` accessor.

## State Additions

```cpp
// In LineFollowerClass private section:
bool _pidFrozen = false;  ///< True on the most recent update() tick where the junction
                          ///< classifier suppressed the P+D mix. Reset to false on the
                          ///< first non-junction tick AND in the disable path.
```

No new builder fields. No new Config constants.

## Update() Control-Flow Sketch

```cpp
void LineFollowerClass::update()
{
    if (!_enabled) return;

    // 1. Resolve cruise from preset + reverse                       [unchanged: lines 163-167]
    int16_t cruise = presetToPwm(_preset);
    if (_reverse) cruise = -cruise;

    // 2. Optional ramp-up modifies cruise                            [unchanged: 169-215]
    // 3. Optional ramp-down modifies cruise                          [unchanged: 217-275]

    // 4. Lost-line dispatch — early return as today                  [unchanged: 277-307]
    if (!_array->isOnline()) {
        _lastErr  = 0.0f;
        _lostLine = true;
        _pidFrozen = false;          // NEW — clear freeze if line is lost mid-junction
        // ... LOST_LINE_ACTION switch (unchanged)
        return;
    }
    _lostLine = false;

    // 5. NEW: junction freeze gate
    const uint8_t junctionRaw = _array->getJunctionType();
    const bool atJunction = (junctionRaw != (uint8_t)JunctionType::NONE
                          && junctionRaw != (uint8_t)JunctionType::UNKNOWN);

    if (atJunction) {
        _pidFrozen = true;
        const int16_t snapped = deadbandSnap(cruise);
        Drive.setSpeed(snapped, snapped);
        // Fall through to distance-gate evaluation below — do NOT early return.
    } else {
        if (_pidFrozen) {
            _lastErr = 0.0f;         // D-term reset on freeze EXIT
            _pidFrozen = false;
        }
        // 6. P+D mix                                                  [unchanged: 314-342]
        const float err  = _array->getError();
        const float dErr = err - _lastErr;
        _lastErr = err;
        float corr = (Config::LINE_KP * err) + (Config::LINE_KD * dErr);
        // clamp + mix + deadbandSnap + Drive.setSpeed (unchanged)
    }

    // 7. Distance gate                                                [unchanged: 350-380]
    if (_distanceTargetActive) { /* unchanged */ }
}
```

Also: in `_applyEnableBuilder`'s disable path (LineFollower.cpp:508-528), add `_pidFrozen = false;` next to the existing `_lastErr = 0.0f; _lostLine = false;` lines so a disable cleanly clears the freeze flag.

## Header API Addition

```cpp
// In LineFollowerClass public section:
/**
 * @brief Whether the most recent update() tick froze the P+D mix due to a
 *        junction classification (LEFT, RIGHT, or T_OR_CROSS).
 * @return true if the freeze branch ran on the last tick; false otherwise
 *         (including whenever the controller is disabled).
 * @note Edge-triggered consumers can poll this between ticks and derive
 *       transitions locally. EventTrace does NOT consume it today; the
 *       accessor exists for Serial debug and future telemetry.
 */
bool isPidFrozen() const;
```

## Error Handling

- **`getJunctionType()` returns a value outside `[0..4]`:** The defensive trigger `junctionRaw != NONE && junctionRaw != UNKNOWN` will treat anything ≥1 and ≠4 (i.e., bytes 5..255) as a junction → freeze. This is safe; mirrors Intersection.cpp:60-62 spirit but does not duplicate its validation (Intersection still validates and clamps). If a future bus corruption returns 0xFF, the freeze will activate spuriously for that tick only and release on the next valid NONE.
- **`isOnline() == false` while `getJunctionType()` returns non-NONE (impossible under current `WROLineArrayClass` but defensive):** Lost-line dispatch wins because the isOnline check is structured to early-return before the freeze gate runs.
- **Disable mid-freeze:** `_applyEnableBuilder` clears `_pidFrozen` along with `_lastErr` and `_lostLine`. Next enable starts with `_pidFrozen == false`; no stale freeze flag.

## Testing Strategy

**Levels:** All on-hardware behavioural tests (this is an Arduino sketch project; no host-side unit harness — see CLAUDE.md "Build & flash"). Wire up a small bring-up sketch or extend an existing one to exercise the freeze paths.

| ID | Test Case | Type | Expected Behavior |
|---|---|---|---|
| TC1 | Drive over a T-intersection at NORMAL cruise, centred on the run-line | Hardware | Chassis crosses smoothly with NO visible wobble; serial heartbeat (or temp `Serial.println` of `isPidFrozen()`) shows `true` for the dwell ticks |
| TC2 | Drive over a LEFT junction (left-branch only) at NORMAL cruise | Hardware | Freeze fires; chassis maintains heading across the branch; resumes PID after the branch ends |
| TC3 | Drive over a RIGHT junction (right-branch only) at NORMAL cruise | Hardware | Freeze fires; symmetric behaviour to TC2 |
| TC4 | Drive over a T-intersection in REVERSE mode | Hardware | Freeze fires; chassis crosses smoothly backward at negated cruise |
| TC5 | Cross a T-intersection while a distance gate is armed and the chassis is inside the ramp-down zone | Hardware | Ramp-down `cruise` continues to lerp; freeze commands `Drive.setSpeed(rampedCruise, rampedCruise)`; distance gate trips on schedule and active brake fires |
| TC6 | Cross a T-intersection inside the ramp-up zone (just after enable) | Hardware | Ramp-up `cruise` continues to lerp; freeze commands symmetric ramped cruise; PID resumes at full ramped cruise after the junction |
| TC7 | Cover all 8 channels with hand mid-segment (force isOnline=false AND classifier momentarily non-NONE) | Hardware | Lost-line dispatch (`Config::LOST_LINE_ACTION`) wins; freeze does not fire; `isPidFrozen()` returns `false` |
| TC8 | Disable the follower mid-junction (call `setEnabled(false)` while frozen) | Hardware | Controller disables with active brake; `isPidFrozen()` returns `false` immediately; re-enable starts with clean state, no kick |
| TC9 | First post-freeze tick: verify no D-term kick by inspecting `getRawError()` and chassis behaviour | Hardware | Chassis does not jerk on freeze exit; `_lastErr` is 0 going into the first PID tick (verify via temp Serial log if needed) |
| TC10 | Intersection edge firing — verify `Intersection.isAtIntersection()` still fires exactly once per junction crossing | Hardware | Edge count from `getEdgeCount()` increments by exactly 1 per physical junction; debounce semantics unchanged |
| TC11 | Forward sweep across +-cross intersection (4-way) | Hardware | Freeze fires for the full cross-bar dwell; chassis exits straight along the run-line |

**Test data:** Use the existing competition mat or a printed test surface with all three junction types (LEFT-only T, RIGHT-only T, full +-cross). Calibrate `WROLineArrayClass` first via the existing `BTN_DOWN` long-press routine.

**Run command:** Arduino IDE → Verify → Upload (board: Arduino UNO R4 WiFi). Then manually drive the chassis over each test surface; observe via OLED + Serial heartbeat.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|---|---|---|---|---|---|
| T1 | Add `_pidFrozen` member + `isPidFrozen()` declaration to LineFollower.h | — | low | `WRO2026/LineFollower.h` | In the `private:` section near `_lastErr` / `_lostLine`, add `bool _pidFrozen = false;` with doxygen. In `public:` near `isLineLost()`, add `bool isPidFrozen() const;` with full doxygen per the "Header API Addition" block in this spec. Update the class header doxygen (top of class) to mention the freeze behaviour in one sentence. Satisfies AC #9, #10. |
| T2 | Implement freeze gate in `LineFollower::update()` and `isPidFrozen()` accessor | T1 | med | `WRO2026/LineFollower.cpp` | (a) Add `LineFollower::isPidFrozen() const { return _pidFrozen; }` near the other accessors. (b) In `update()`, after the lost-line branch's early return and before the existing `_lastErr = err;` line, insert the freeze gate per the "Update() Control-Flow Sketch" in this spec: call `_array->getJunctionType()` once into a const `uint8_t`, compute `atJunction = (raw != NONE && raw != UNKNOWN)` using the `JunctionType` enum from Intersection.h, wrap the P+D block in if/else, command `Drive.setSpeed(snapped, snapped)` on freeze ticks where `snapped = deadbandSnap(cruise)`, reset `_lastErr = 0.0f` on the first non-frozen tick after a frozen tick. The distance-gate code below the P+D block stays where it is and runs unconditionally. Update the `update()` method-level doxygen to document the new freeze branch. Satisfies AC #1, #2, #3, #4, #5, #6, #9. |
| T3 | Clear `_pidFrozen` in `_applyEnableBuilder` disable path AND in the lost-line branch | T2 | low | `WRO2026/LineFollower.cpp` | In `_applyEnableBuilder` disable path (around the existing `_lastErr = 0.0f; _lostLine = false;` lines at ~512-516), add `_pidFrozen = false;`. In `update()`'s lost-line branch (around line 281 where `_lastErr = 0.0f;` lives today), also add `_pidFrozen = false;` so a freeze flag cannot leak across a lost-line transition either. Satisfies AC #8, #10. |
| T4 | Verify Intersection.update() and EventTrace remain untouched | T2 | low | `WRO2026/Intersection.cpp`, `WRO2026/EventTrace.cpp` | Read both files and confirm no changes are needed; this is a verification task, not an edit. Confirms AC #7. If verification reveals any unexpected coupling, raise it back to T2 rather than silently editing. |
| T5 | Update DECISIONS.md with the freeze design | T1, T2 | low | `DECISIONS.md` | Add a short "LineFollower PID freeze during junction dwell" section documenting: the centroid-skew defect that motivated the change, the broadened trigger scope (LEFT/RIGHT/T_OR_CROSS, not just T_OR_CROSS as issue #22 originally proposed), the symmetric-cruise + motor-bias output choice, the instant-enter/instant-exit debounce decision, the D-term reset on exit, and the explicit no-EventTrace choice. Style: match the existing entries (decision header + rationale + alternatives considered). |
| T6 | On-hardware behavioural test pass | T2, T3 | high | (none — hardware test) | Run TC1-TC11 from the Testing Strategy table on the competition mat with at least one LEFT-only T, one RIGHT-only T, and one +-cross. Document results in a brief comment on issue #22 (which junction types tested, observed behaviour, any wobble residuals). If any test fails, file the deviation back to T2 or T3 as appropriate before merging. Satisfies all acceptance criteria via observation. |

T2 is medium risk (modifies controller hot path; a wrong if/else placement could starve the distance gate or leak a freeze flag). T6 is high risk because it is the only verification path — there is no host-side test harness, so a regression that compiles will only surface on hardware.

## Notes for Implementer

- **Do not call `_array->getJunctionType()` twice per tick from inside LineFollower.** A single local `const uint8_t junctionRaw = _array->getJunctionType();` is enough. With `LINE_ARRAY_TICK_CACHE_ENABLED == false` (today's default), each call is a ~9 ms I²C transaction — duplicating it would visibly slow the loop.
- **Intersection.update() will also call `getJunctionType()` this same tick.** That's the existing double-read situation acknowledged in PR #20 review and in issue #22's open questions; tick-scoped caching is the planned mitigation but is out of scope for this change.
- **Do not bypass `deadbandSnap()` on the freeze output.** The P+D path snaps both L and R via `deadbandSnap()` before `Drive.setSpeed`. The freeze path must do the same so a `cruise` value inside the deadband range (rare but possible during ramp-up) does not get silently dropped by `Drive.setSpeed`'s internal re-snap into a stop.
- **Existing comment at LineFollower.cpp:336-339 already documents that `Drive.setSpeed(L, R)` applies motor-bias calibration automatically.** No new code is needed to "preserve motor bias during freeze" — calling `Drive.setSpeed(cruise, cruise)` is sufficient.
- **The `JunctionType` enum lives in Intersection.h.** LineFollower.cpp already includes "Intersection.h" (line 10), so casting `_array->getJunctionType()` against `JunctionType::NONE` / `JunctionType::UNKNOWN` is in scope. No new include needed.
- **Rollback plan:** This change is local to LineFollower; reverting the commit fully restores prior behaviour. No data migration, no Config changes to undo.
- **Why no debounce on exit:** A single noisy NONE mid-junction would un-freeze for one tick → run P+D on a still-skewed err sample → immediately re-freeze. The visible wobble from one bad PID tick is roughly what we're trying to eliminate. If field tests show flicker, the cheapest fix is 1-tick release hysteresis (one additional `uint8_t` counter); revisit only if observed.
