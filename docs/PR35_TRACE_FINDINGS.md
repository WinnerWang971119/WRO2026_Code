# PR #35 Trace-Review Findings — Tasks framework + `Mission.task()`

This document catalogs the results of an adversarially-verified behavior-tracing review run against PR #35
(the Tasks framework + `Mission.task()` verb). The trace discovers what each capability can *do*, walks the
code path across files, and verifies every candidate before it lands here. Note: the trace covered the **whole
repository**, not just the PR diff — so most findings below are in pre-existing modules the PR merely surfaced.

**Run stats:** 82 confirmed + 21 to-review (uncertain) survived verification; 44 candidates were refuted and
dropped. After de-duplicating near-identical findings (the same defect reported under two capability labels or at
two adjacent call sites), **73 confirmed** (findings 1-73) + **21 review-manually** (findings 74-94) remain. Nine
source entries were folded into seven host findings — each merge is flagged inline with a *(merged)* note listing
the capability labels / call sites it was found under.

**Severity legend:**
- `bug` — confirmed defect with clear runtime impact.
- `suspicious` — very likely wrong; mechanism is sound but the trigger or blast radius needs a human eye.
- `possible` — plausible defect or robustness gap (latent contract hole, missing guard, edge-case-only).

**Scope tags:**
- `PR-surface` — file is in PR #35's changed set (`Config.h`, `Mission.cpp/.h`, `Tasks.cpp/.h`, `TasksTypes.h`,
  `WRO2026.ino`, `test/*`).
- `pre-existing` — the trace surfaced it but PR #35 did not introduce it (Drive, LineFollower, PathTracker,
  WROLineArray, EventTrace, Intersection, Localization, FieldMap, and all of `tools/path_planner/`).

## How to use this doc

Each confirmed finding has a plain-English **What's wrong**, then 2-3 **Options** (A/B/C) with tradeoffs and a
one-line **Recommendation**. A blank **Decision:** line follows each one — fill it in. Findings are numbered
sequentially (1..N) for easy reference. The **Review manually** section is lighter: explanation + a "what to
check" line, with options only where an obvious fix exists. Nothing here has been changed in source — this is
a read-only report.

---

## Table of contents

- [Confirmed — PR-surface](#confirmed--pr-surface)
  - Bugs: 1-4
  - Suspicious: 5
  - Possible: 6-12
- [Confirmed — pre-existing (out of PR scope)](#confirmed--pre-existing-out-of-pr-scope)
  - Firmware — Drive: 13-16
  - Firmware — LineFollower: 17-21
  - Firmware — WROLineArray: 22-24
  - Firmware — Intersection: 25-26
  - Firmware — EventTrace: 27-28
  - Firmware — Localization: 29-31
  - Firmware — PathTracker: 32
  - Firmware — FieldMap / DirectionPolicy / WROMath: 33-35
  - Offline — build_routes: 36
  - Offline — graph_builder: 37-41
  - Offline — planner: 42-43
  - Offline — compiler: 44-45
  - Offline — validator: 46-47
  - Offline — header_writer: 48-50
  - Offline — pipeline_config: 51
  - Offline — visualizer: 52-53
  - Offline — sim/runtime: 54-55
  - Offline — pick_xy: 56-60
  - Offline — editor: 61-67
  - Offline — migrate / loader: 68-70
  - Offline — tests: 71-73
- [Review manually (uncertain)](#review-manually-uncertain)
  - 74-94

---

## Confirmed — PR-surface

### Bugs

#### 1. `task()` never sets `_lastPushOk`; an overflowed task lets a later direction modifier corrupt the previous `goTo`

- **Location:** `WRO2026/Mission.h:372`
- **Severity:** bug   **Scope:** PR-surface
- **Capability:** mission DSL sequencer
- **Traced path:** `task<Args...>() -> _push(s) [return discarded] -> setRouteDirection()/setArrivalDirection() guards pass on stale _lastPushOk -> _queue[_count-1] is the PREVIOUS goTo -> its direction is rewritten`

**What's wrong:** Every other step-pushing verb assigns `_lastPushOk = _push(s)` (`goTo` Mission.cpp:59,
`wait`:68, `print`:77, `beep`:87, `led`:99). The variadic `task()` template calls `_push(s)` but throws the
return value away and never touches `_lastPushOk` (Mission.h:372). The member's own doc (Mission.h:309-316)
states the invariant: it must be reset by every push so a chained modifier can only ever target the
immediately-preceding push. `setRouteDirection`/`setArrivalDirection` guard on `_lastPushOk` *precisely* to
avoid retargeting an older step after an overflow-dropped push. When the 64-slot queue is full, a `task(...)`
push is silently dropped, `_count` does not advance, and `_lastPushOk` keeps the *prior* verb's value. If that
prior verb was a successful `goTo`, a following `.setRouteDirection(REAR)` passes both guards and silently
flips the unrelated `goTo`'s direction — exactly the corruption the guard exists to prevent.

**Options:**
- **A —** Set `_lastPushOk = _push(s);` in `task()` (Mission.h:372), matching every other verb. _(One-line fix; restores the documented invariant. Best.)_
- **B —** Set `_lastPushOk = false` unconditionally at the top of `task()` so a dropped task can never be a modifier target, even on a successful push. _(Safer against future modifiers but breaks any intent to chain a modifier onto a task step — though no task modifier exists today.)_
- **C —** Leave as-is. _(Live `buildMission()` never fills the queue today, so it is latent — but it is a one-line, zero-risk fix.)_

**Recommendation:** A — it is the minimal change that restores the stated invariant and matches all sibling verbs.

**Decision:** A

#### 2. LED step / fault-flash writes invalid index 0, lighting only one of the two on-board RGB LEDs

- **Location:** `WRO2026/Mission.cpp:211`
- **Severity:** bug   **Scope:** PR-surface
- **Capability:** cooperative tick / main loop orchestration
- **Traced path:** `cooperativeTick() -> Mission.update() (LED step / _enterFaultBeeping / FAULT_BEEPING) -> _setLeds() -> MiniR4.LED.setColor(0, r, g, b) [rejected] then setColor(1, ...)`

**What's wrong:** `_setLeds()` calls `MiniR4.LED.setColor(0, r, g, b)` then `setColor(1, r, g, b)`. The library
(`MiniR4LED::setColor`, MiniR4LED.h:58) rejects any index outside 1..2: `if (idx < 1 || idx > 2) return false;`.
So the `setColor(0,...)` call is silently dropped and only LED index 1 is ever written — the second on-board RGB
LED (index 2) is never touched. The two on-board LEDs are addressed as 1 and 2 (index 0 is invalid). Every LED
behavior routed through `cooperativeTick` half-lights: the blue arm-flash, the green/white step LEDs, and the
documented red dual-LED fault flash (WRO2026.ino:29-30) all light only one LED.

**Options:**
- **A —** Change the calls to `setColor(1, r, g, b)` and `setColor(2, r, g, b)`. _(Correct, minimal; this is the exact fix the trace identifies.)_
- **B —** Loop `for (uint8_t i = 1; i <= 2; ++i) MiniR4.LED.setColor(i, r, g, b);`. _(Same effect, marginally more future-proof if LED count ever changes; slightly more code.)_
- **C —** Leave as-is. _(Not legitimate — the fault indicator is a safety cue the spec promises in full.)_

**Recommendation:** A — the two-LED hardware is fixed at indices 1 and 2; the explicit pair is the clearest fix.

**Decision:** A

#### 3. Shipped `buildMission()` references node names absent from the generated routes table — mission faults on the first `goTo`

- **Location:** `WRO2026/WRO2026.ino:161`
- **Severity:** bug *(reported as suspicious; presented here as a hard, reproducible fault on every run)*   **Scope:** PR-surface
- **Capability:** mission DSL sequencer
- **Traced path:** `buildMission() -> Mission.goTo("B") -> FieldMap::findNodeByName("B") == NODE_NONE (0xFF) -> Mission.update() GOTO ENTER guard s.goal==NODE_NONE -> _enterFaultBeeping()`

**What's wrong:** `buildMission()` enqueues `goTo("B")`, `goTo("G")`, `goTo("C")`, `goTo("D")`, `goTo("A")`. The
committed `routes.cpp` NODES array (routes.cpp:15-20, `N_NODES=4`) only defines `"Inter1"`, `"Inter2"`,
`"Start"`, `"finial"`. None of `A`/`B`/`C`/`D`/`G` resolve, so `findNodeByName` returns `NODE_NONE=0xFF` for
each. The first GOTO's ENTER guard (`s.goal == NODE_NONE || s.goal >= N_NODES`) is true, pushes `NAV FAULT BAD`,
and drops straight into `FAULT_BEEPING`. The `.ino` docstrings (lines 12, 22) also call `START_NODE` `'A'`, but
`START_NODE=2` is `"Start"`. The sequencer handles the unknown node correctly — the defect is that the shipped
demo mission and its docs are stale relative to the auto-generated graph.

**Severity note:** kept as `bug` impact in the body (it faults deterministically on arm) though the JSON tagged
it `suspicious`; severity field not changed.

**Options:**
- **A —** Rewrite `buildMission()` to use the real node names (`"Start"`, `"Inter1"`, `"Inter2"`, `"finial"`) and fix the `.ino` docstrings. _(Makes the demo actually run; requires deciding the intended route across the 4-node graph.)_
- **B —** Regenerate `routes.*` from a `field_data` whose NODES match the `A..G` naming the mission assumes. _(Only correct if the `A..G` field is the real competition field; otherwise it papers over a stale mission.)_
- **C —** Leave as-is and document that `buildMission()` is a placeholder to be authored per-run. _(Legitimate only if no one expects the committed mission to drive; risky as a shipped default since arm-on-button faults immediately.)_

**Recommendation:** A — align the mission and docs to the generated graph so a button-arm produces motion, not an instant fault.

**Decision:** Do nothing.

#### 4. EventTrace render wipes the calibration menu off the OLED on the first tick after entry

- **Location:** `WRO2026/WRO2026.ino:313`
- **Severity:** bug   **Scope:** PR-surface
- **Capability:** on-board calibration menu
- **Traced path:** `menu-entry -> EventTrace.pushEvent("CAL MENU") sets _dirty=true -> renderCalMenu() paints menu -> return -> next loop() -> cooperativeTick() -> EventTrace.update() sees _dirty -> _render() clearDisplay()+repaints event ring over the menu`

**What's wrong:** On menu entry the code calls `EventTrace.pushEvent("CAL MENU")` (line 313), which sets
EventTrace's `_dirty=true`, then `renderCalMenu()` (line 318) paints the menu directly. `renderCalMenu()` does
not clear the dirty flag, and the entry path returns (line 329) without running `EventTrace.update()`. On the
very next `loop()` iteration `cooperativeTick()` runs first and calls `EventTrace.update()`; `_dirty` is still
true, so `_render()` does `clearDisplay()` and repaints the 4-entry event ring (`"CAL MENU"`, `"BOOT OK"`, …)
*on top of* the menu. The menu and cursor are erased within ~one tick of opening and stay erased until the
operator blindly presses BTN_DOWN (the only other call to `renderCalMenu`, line 256). The operator cannot see
the default selection (LINE), and a BTN_UP confirm at that point dispatches LINE while the screen shows the
event log.

**Options:**
- **A —** Clear EventTrace's dirty flag when entering the menu (e.g. add `EventTrace.markClean()` / suppress its render) and make EventTrace skip rendering while `_calMenuState == OPEN`. _(Proper fix: the menu owns the OLED while open; needs a small EventTrace API or a render-suppress gate.)_
- **B —** Don't call `EventTrace.pushEvent("CAL MENU")` on entry (log to Serial only); the menu then never gets clobbered by its own dirty flag. _(Smallest change, but other events pushed while the menu is open — none today — would still clobber it; loses the menu-entry trace line.)_
- **C —** Re-paint the menu every loop in the OPEN state (call `renderCalMenu()` unconditionally) so EventTrace's wipe is immediately overwritten. _(Brute-force; causes flicker and fights EventTrace every tick.)_

**Recommendation:** A — gate EventTrace rendering off while the menu owns the display; it is the only fix that keeps both subsystems correct.

**Decision:** A

### Suspicious

#### 5. Holding BTN_UP during menu entry immediately confirms the default LINE selection

- **Location:** `WRO2026/WRO2026.ino:260`
- **Severity:** suspicious   **Scope:** PR-surface
- **Capability:** on-board calibration menu
- **Traced path:** `menu-entry waits only for BTN_DOWN release (323-326), sets _btnUpPrev=false -> next loop OPEN handler reads btnUpNow=true, _btnUpPrev=false -> rising edge -> confirm + dispatch LINE`

**What's wrong:** The IDLE→OPEN entry path (lines 320-328) waits only for BTN_DOWN to release
(`while (MiniR4.BTN_DOWN.getState())`) and then forces `_btnUpPrev=false`. It never samples or waits on BTN_UP.
If BTN_UP is held when entry completes (operator pressing both buttons, or a thumb resting on BTN_UP), the next
OPEN pass sees `btnUpNow && !_btnUpPrev` as true (line 260) and treats it as a confirm — dispatching
`LineArray.calibrate()` on the default LINE item with no deliberate press. Note the IDLE arm path guards against
simultaneous BTN_DOWN (`_btnDownPressStartMs == 0`, line 351), but the OPEN confirm has no equivalent guard
against a button already down on entry.

**Options:**
- **A —** On menu entry, wait for *both* buttons to release before clearing `_btnUpPrev` (change the entry release loop to `while (BTN_DOWN.getState() || BTN_UP.getState())`, mirroring the confirm-exit loop at lines 285-288). _(Directly closes the hole; matches the existing exit-path pattern.)_
- **B —** Prime `_btnUpPrev` from the actual BTN_UP level at entry instead of forcing it false, so a held button is not seen as a fresh edge. _(Also correct; slightly subtler but avoids the busy-wait extension.)_
- **C —** Leave as-is. _(A two-button fumble is plausible and triggers a multi-second blocking sweep; not advisable to ignore.)_

**Recommendation:** A — reuse the both-buttons-released pattern the exit path already uses, for consistency.

**Decision:** A

### Possible

#### 6. BEEP step truncates the buzzer tone to 16 bits but gates step advance on the full 32-bit duration

- **Location:** `WRO2026/Mission.cpp:375`
- **Severity:** possible   **Scope:** PR-surface
- **Capability:** mission DSL sequencer
- **Traced path:** `Mission.update() BEEP ENTER -> _beep(s.freq_hz, (uint16_t) s.duration_ms) [truncated] ; ACTIVE gate -> now - _stepStart_ms >= s.duration_ms [full uint32_t]`

**What's wrong:** The public verb is `beep(uint16_t freq_hz, uint32_t duration_ms)` and `Step` stores
`duration_ms` as `uint32_t`. On ENTER the tone duration is cast to `uint16_t`
(`_beep(s.freq_hz, (uint16_t) s.duration_ms)`, Mission.cpp:375; `_beep` itself takes a `uint16_t` too), but the
advance gate (Mission.cpp:379) compares against the full `uint32_t s.duration_ms`. For any `duration_ms > 65535`
the audible tone stops after `duration_ms % 65536` ms while the step still blocks for the full duration — buzzer
and step clock diverge (e.g. `beep(1000, 70000)` goes silent at ~4.4 s but the step holds for 70 s).

**Options:**
- **A —** Clamp the gate to the truncated value, or better, clamp `duration_ms` to 65535 at the `beep()` verb and document the ceiling. _(Makes audio and step length agree; caps max beep at ~65 s, which is plenty.)_
- **B —** Widen `_beep` / the buzzer call to accept `uint32_t` if the library supports it, removing the cast. _(Only valid if `MiniR4Buzzer::Tone` accepts >16-bit ms; otherwise the truncation is forced by the library.)_
- **C —** Leave as-is and document that beep durations above 65535 ms are unsupported. _(Acceptable — no real mission needs a >65 s beep — but the silent divergence is a footgun.)_

**Recommendation:** A — clamp at the verb so the two clocks can never disagree; a >65 s tone is not a real use case.

**Decision:** A

#### 7. A queue-time bad task name latches a build error that cannot be re-armed without a power cycle

- **Location:** `WRO2026/Mission.cpp:146`
- **Severity:** possible   **Scope:** PR-surface
- **Capability:** mission DSL sequencer
- **Traced path:** `Mission.task("typo") sets _buildError -> start() refuses, _enterFaultBeeping() -> FAULT_AWAIT_RESET -> BTN_DOWN clears _buildError+IDLE ; but .ino _missionArmed stays true so buildMission()/start() never run again`

**What's wrong:** `task()` validates the registry name at queue time and sets `_buildError=true` on a miss
(Mission.h:359-366). `start()` then refuses and drops into `_enterFaultBeeping` (Mission.cpp:146-152). The
operator can ack via BTN_DOWN (FAULT_AWAIT_RESET clears `_buildError` and returns to IDLE). But in the `.ino`,
`_missionArmed` is set true the moment `buildMission()` is invoked (WRO2026.ino:360-362) and is never reset,
and the BTN_UP arm path is gated by `!_missionArmed` (line 351). So after acking a build-error fault, the
operator cannot re-arm this power cycle — a single mistyped task name bricks the run until reboot, even though
the sequencer state was cleanly reset.

**Options:**
- **A —** Reset `_missionArmed = false` on the fault-acknowledge path (when BTN_DOWN clears a build-error fault), so BTN_UP can re-arm. _(Restores recoverability; must ensure re-arm re-runs `buildMission()` cleanly.)_
- **B —** Only set `_missionArmed = true` *after* `Mission.start()` succeeds (not when `buildMission()` is invoked), so a refused start leaves the operator able to retry. _(Cleaner state model; re-press rebuilds and retries.)_
- **C —** Leave as-is, treating "one buildMission per power cycle" as intended. _(Matches a documented intent, but a typo costing a reboot is harsh for a competition pit; the typo is also caught at compile-author time, lowering real-world odds.)_

**Recommendation:** B — gating `_missionArmed` on a successful `start()` is the most principled fix and naturally covers this case.

**Decision:**C, one mission per PC

#### 8. `_aborted` latch is write-only in firmware: `abort()` sets it but nothing reads `isAborted()`

- **Location:** `WRO2026/Tasks.cpp:150`
- **Severity:** possible   **Scope:** PR-surface
- **Capability:** tasks framework
- **Traced path:** `MissionClass::abort() -> Tasks.abort() sets _aborted=true (Tasks.cpp:150) -> isAborted() (Tasks.cpp:161) never called by any production .cpp/.ino`

**What's wrong:** Across the firmware, `isAborted()` is referenced only in comments (Mission.cpp:159, 177-181)
and in the host test mirror; no production path consumes it. `abort()` sets `_aborted`, `clearAbort()`/`begin()`
clear it, but because nothing polls it the sticky-abort latch changes no runtime behavior. The class comment and
Mission.cpp:177-181 acknowledge the flag is "wired but subsystems do not yet poll it" — the actual stop comes
from `PathTracker.abort()` / `Drive.stop(true)`. This is a deliberately dormant feature, but as shipped the
latch is dead state, and the first real cooperative task body that relies on it is easy to mis-wire.

**Options:**
- **A —** Leave it dormant but add a one-line comment at the `isAborted()` definition pointing to the intended first consumer, and keep it out of any "abort is enforced" claims. _(Honest documentation of a staged feature; zero behavior change.)_
- **B —** Wire the existing task bodies (`demoEcho`/`debugYield`) to poll `Tasks.isAborted()` and bail, so the latch is exercised end-to-end and the contract is proven before the first real task lands. _(Validates the mechanism now; minor churn in stub bodies.)_
- **C —** Remove the latch until a consumer exists. _(Avoids dead state but throws away a deliberately-staged hook; not recommended given the framework is mid-build.)_

**Recommendation:** A (with B if a quick task-body poll is cheap) — keep the staged hook, just stop it from reading as enforced.

**Decision:** A

#### 9. Button `getState()` swallows the I2C result code, so a transient read failure registers as a spurious edge

- **Location:** `WRO2026/WRO2026.ino:350`
- **Severity:** possible   **Scope:** PR-surface
- **Capability:** cooperative tick / main loop orchestration
- **Traced path:** `MiniR4.BTN_UP.getState()/BTN_DOWN.getState() -> mmL.GetButtonState(_id, state) [RESULT discarded] -> rising-edge logic (btnNow && !prev)`

**What's wrong:** `MiniR4BTN::getState()` (MiniR4BTN.h:34-39) calls `mmL.GetButtonState(_id, state)` but ignores
the returned `MMLower::RESULT`; on an I2C failure `state` stays its default `false`. The `.ino` button handling
is entirely rising-edge based and the 2 s-hold detector compares against the previous sample. A single failed
read during a held press therefore reads as "released": it can reset `_btnDownPressStartMs` to 0 mid-hold
(line 336, restarting the 2 s timer) and, when the next read succeeds as "pressed", manufacture a false rising
edge that cycles the menu cursor, confirms an item, or arms the mission.

**Options:**
- **A —** Add a small debounce (require N consecutive equal samples before accepting a level change) around the button reads in `loop()`. _(Absorbs single-sample glitches for all button paths; modest code.)_
- **B —** Change `getState()` (or wrap it locally) to return the last good level when the I2C result is non-OK, rather than defaulting to `false`. _(Fixes at the source, but `getState()` is a library method — wrap it in the `.ino` instead of editing the vendor header.)_
- **C —** Leave as-is. _(Per-tick probability is low, but a spurious arm or cal-dispatch from noise is a real safety surprise; debounce is cheap.)_

**Recommendation:** A — a few-sample debounce in the `.ino` covers every button path (this finding plus #5, #10, #11) at once.

**Decision:** C

#### 10. BTN_UP held at boot arms the mission on the first loop because `_btnUpPrev` is not primed

- **Location:** `WRO2026/WRO2026.ino:351`
- **Severity:** possible   **Scope:** PR-surface
- **Capability:** cooperative tick / main loop orchestration
- **Traced path:** `setup() (no BTN priming) -> first loop() pass -> btnUpNow=true, _btnUpPrev=false -> rising edge -> zeroHeadingFromCurrent() + buildMission()`

**What's wrong:** `_btnUpPrev` is statically initialized to `false` (WRO2026.ino:83) and `setup()` never primes
it from the actual button level. On the first `loop()` pass, if BTN_UP happens to be physically held (operator
powered on with a thumb on it, or is mid-press), `btnUpNow && !_btnUpPrev && !_missionArmed && _btnDownPressStartMs == 0`
is satisfied with no real human edge, immediately zeroing the IMU offset and building+starting the mission.
`_btnDownPrev` has the same un-primed pattern for the menu path.

**Options:**
- **A —** In `setup()` (after `MiniR4.begin()`), prime `_btnUpPrev = MiniR4.BTN_UP.getState()` and `_btnDownPrev = MiniR4.BTN_DOWN.getState()`. _(One-line-each fix; a held-at-boot button no longer reads as a fresh edge.)_
- **B —** Require the button to be seen released at least once before the first arm is accepted (a `_armReady` latch set once `getState()` returns false). _(More robust against boot-time holds; a little more state.)_
- **C —** Leave as-is. _(Powering on with a finger on BTN_UP is plausible and zeros the IMU against a wrong pose; cheap to fix.)_

**Recommendation:** A — prime the edge-detect state in `setup()`, the standard fix for un-primed boot edges.

**Decision:** C

#### 11. 2-second hold timer resets on any single dropped BTN_DOWN sample; menu nav is undebounced

- **Location:** `WRO2026/WRO2026.ino:336` (also covers the undebounced nav/confirm reads at `:252`)
- **Severity:** possible   **Scope:** PR-surface
- **Capability:** on-board calibration menu
- **Traced path:** `IDLE -> btnDownNow=getState() -> else branch resets _btnDownPressStartMs=0 on any false read ; OPEN -> (btnDownNow && !_btnDownPrev) cursor advance with no debounce`

**What's wrong:** The 2 s menu-entry hold uses raw, undebounced reads. Because `getState()` returns `false` on
an I2C failure (see #9), any single dropped/bounced sample during the hold hits the else branch (line 336) and
resets `_btnDownPressStartMs=0`, discarding all accumulated hold time — the operator must hold cleanly for 2 s.
The same `false`-on-error behavior also lets the release-wait loops (lines 323-326, 285-288) exit early. In the
OPEN state, cursor cycling (line 252) and confirm (line 260) are single-sample rising-edge detectors with no
debounce, so one spurious "pressed" sample advances the cursor or confirms (e.g. dispatching MOTOR, which spins
the wheels).

**Options:**
- **A —** Apply the same N-sample debounce from #9 to all BTN reads; the hold timer and nav edges then tolerate single glitches. _(Single mechanism fixes #9, #10, #11; recommended once.)_
- **B —** Make the hold timer tolerant of brief drops specifically (e.g. only reset after K consecutive released samples) without a full debounce layer. _(Narrower; still leaves nav/confirm undebounced.)_
- **C —** Leave as-is. _(Intermittent menu-open failures and stray cursor jumps are annoying but non-destructive except for an accidental MOTOR/LINE dispatch.)_

**Recommendation:** A — fold into the shared button debounce; it is the same root cause as #9 and #10.

**Decision:** C

#### 12. README documents 3 ctest cases but CMake registers 7

- **Location:** `WRO2026/test/README.md:31`
- **Severity:** possible   **Scope:** PR-surface
- **Capability:** C++ host-compile test harness
- **Traced path:** `README "Quick start" expected output -> CMakeLists.txt add_test x7`

**What's wrong:** README.md (lines 8-13, 24-32) lists only three test executables and shows expected ctest output
ending in "100% tests passed, 0 tests failed out of 3." CMakeLists.txt actually registers seven `add_test()`
cases: `line_follower_overshoot`, `path_tracker_passthrough_on/off`, `path_tracker_bidirectional_on/off`,
`test_tasks_dispatch`, and `tasks_overflow_compile_fail`. The README's "What's mocked" / "Adding a new test"
sections were never updated when the bidirectional and tasks targets were added.

**Options:**
- **A —** Update README.md to list all seven tests and the correct expected output. _(Documentation fix; matches the harness.)_
- **B —** Regenerate the test list in the README from CMake (or a comment pointing at CMakeLists as the source of truth). _(Avoids future drift; more setup.)_
- **C —** Leave as-is. _(An operator running the documented `ctest` sees 7 not 3 and may think their checkout is misconfigured.)_

**Recommendation:** A — refresh the README to the current seven-test harness.

**Decision:** A

---

## Confirmed — pre-existing (out of PR scope)

### Firmware — Drive

#### 13. Profiled move ending at non-zero endSpeed never stops the chassis; `isIdle()` reports idle while wheels keep spinning

- **Location:** `WRO2026/Drive.cpp:655`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** drivebase control
- **Traced path:** `DriveDistBuilder dtor (endSpeed defaults to cruise when brake=false) -> _enqueue -> update() -> _updateProfile() frac>=1.0 -> _profileEndSpd != 0 branch issues NO Move(0,0)/brake, only sets _moveDone=true, _profileState=IDLE -> isIdle() returns true while the last cruise setpoint persists`

**What's wrong:** In `_updateProfile()` the completion branch for `_profileEndSpd != 0` (lines 655-663)
deliberately issues no motor command and no brake, leaving the previous tick's cruise setpoint latched on the
STM32. It sets `_moveDone=true` and `_profileState=IDLE`, so `isIdle()` reports idle. The design comment assumes
"the next queued move's first tick overwrites it," but nothing enforces a follow-up exists. When the queue is
empty (a single `Drive.driveDist(30).setBrake(false).async()`, or a PathTracker RAW_FLAT last segment at
PathTracker.cpp:1001-1006), the wheels keep driving at cruise forever while every state accessor says idle — the
planner proceeds believing the robot stopped while it rolls off the node.

**Options:**
- **A —** When the profile completes with `_profileEndSpd != 0` *and the queue is empty*, fall through to the brake/stop path instead of latching the cruise setpoint. _(Closes the runaway; preserves the seamless-handoff optimization when a next move exists.)_
- **B —** Require a non-zero `endSpeed` move to always be followed by another queued command at enqueue time (reject a trailing cruise-ending move). _(Stronger contract but rejects a legitimate "keep rolling into the next phase" intent and shifts the error earlier.)_
- **C —** Leave as-is and document that a cruise-ending move must always be chained. _(Dangerous as a default — the failure is a silent runaway with `isIdle()` lying; not recommended.)_

**Recommendation:** A — brake on completion when nothing is queued; it removes the runaway without losing the handoff fast-path.

**Decision:** A — brake when the queue is empty (silent runaway; `isIdle()` lies).

#### 14. Non-profiled SYNC/GYRO moves and continuous primitives never clamp power to `PWM_MAX`

- **Location:** `WRO2026/Drive.cpp:195`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** drivebase control
- **Traced path:** `DriveDistBuilder dtor -> _enqueue -> _dispatchNext -> _execDriveDistSync/_execDriveDistGyro/_execDriveTimeSync/_execDriveTimeGyro (pass raw power to MoveSyncDegs/MoveGyroDegs, no deadbandSnap/clamp)`

**What's wrong:** The RAW path, the trapezoidal path, and `setSpeed` all route PWM through `deadbandSnap()`
which caps magnitude at `Config::PWM_MAX` (95). But the non-profiled SYNC/GYRO executors (lines 195, 211, 243,
258) and the continuous `setSpeedSync`/`forwardGyro` forward the caller's raw `power` straight to the lower-MCU
API, which the vendor header documents as expecting only [-100,100] with no visible clamp. A call like
`Drive.driveDist(30).setMode(SYNC).setPower(120)` or `Drive.setSpeedSync(120,120)` reaches the STM32 unclamped,
unlike every other Drive path.

**Options:**
- **A —** Route these executors' power through `deadbandSnap()` (or an explicit `PWM_MAX` clamp) like the other paths. _(Uniform clamping everywhere; trivial.)_
- **B —** Validate `power` at the builder/setter and reject out-of-range values with a fault. _(Surfaces the error to the caller instead of silently clamping; more disruptive.)_
- **C —** Leave as-is, trusting firmware saturation. _(Behavior then depends on undocumented STM32 saturation — inconsistent with the rest of Drive.)_

**Recommendation:** A — clamp at these call sites for parity with every other Drive path.

**Decision:** A — clamp SYNC/GYRO + continuous power via `deadbandSnap` for parity with every other Drive path.

#### 15. Triangle-fallback peak-reduction branch is effectively unreachable; the velocity-integral invariant is not maintained

- **Location:** `WRO2026/Drive.cpp:856`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** drivebase control
- **Traced path:** `_execProfileMove() -> half-of-total caps applied to accelMs/decelMs (and accelCm/decelCm) BEFORE the triangle test -> if (accelMs+decelMs > total) (856) / if (accelCm+decelCm > totalCm) (911) essentially never true`

**What's wrong:** For both the time variant (843-878) and distance variant (902-932), each ramp is first clamped
to `total*0.5` / `totalCm*0.5`. After that, their sum is at most `total`, so the subsequent `accel+decel > total`
triangle test is only reachable via float rounding noise. The peak-reduction code (and its rescale) is therefore
dead: when ramps would otherwise overrun, the profile runs at full cruise peak with zero plateau, and the doc
comment claiming a reduced peak keeps the integral equal to the requested distance/duration is false. (Distance
moves self-correct because `frac` is encoder-based; time moves simply cover less distance.)

**Options:**
- **A —** Compute the triangle test *before* applying the half caps (or remove the half caps and let the triangle branch own the overrun case). _(Restores the documented reduced-peak profile; needs care to keep the integral correct.)_
- **B —** Keep the half caps, delete the now-dead peak-reduction branch, and update the doc to describe the actual full-cruise-spike behavior. _(Honest about reality; loses the smoother profile on short moves.)_
- **C —** Leave as-is. _(Short moves get a cruise spike instead of a triangle; cosmetic on most moves, but the doc is wrong.)_

**Recommendation:** B if the spike is acceptable on short moves; A if the smooth triangle matters — decide based on whether short profiled moves are common on the field.

**Decision:** A — run the triangle test before the half-caps so short moves get the smooth reduced-peak profile (keep the distance integral correct).

#### 16. A single bad builder knob (`accel<=0`) faults and aborts the entire queued mission at dispatch time

- **Location:** `WRO2026/Drive.cpp:757`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** drivebase control
- **Traced path:** `DriveDistBuilder dtor (no validation of _accel) -> _enqueue(PROFILE_DIST, accel<=0) -> update() -> _dispatchNext -> _execProfileMove returns 0x01 -> _dispatchNext latches DISPATCH_FAILED, stops chassis, _clearQueue() wipes all remaining commands`

**What's wrong:** `_execProfileMove` rejects `accel <= 0` with rc 0x01 (757-762). Builders enqueue without
validating `_accel`, so the rejection only surfaces at the front of the queue. `_dispatchNext` treats any
non-zero rc as a hard fault: stop, latch `DISPATCH_FAILED`, `_clearQueue()` — discarding every other valid
queued command. One malformed profile aborts the whole pending batch; there is no per-command skip.

**Options:**
- **A —** Validate `_accel > 0` in the builder (at enqueue) and reject/fall back there, so a bad knob never poisons the queue. _(Fails fast at the author's call site; clearest.)_
- **B —** In `_dispatchNext`, on a per-command validation failure, skip that one command (log it) instead of clearing the whole queue. _(Keeps the rest of the mission; changes fault semantics — needs a deliberate "skip vs fault" policy.)_
- **C —** Leave as-is. _(One typo'd accel drops the whole maneuver mid-mission with a fault; harsh but contained.)_

**Recommendation:** A — validate at enqueue so the error is caught before it can take the queue down with it.

**Decision:** A — validate `accel > 0` at the builder so one bad knob can't take down the whole queue.

### Firmware — LineFollower

#### 17. Distance gate cannot trip while the line is lost (non-overshoot), forcing a 10 s timeout

- **Location:** `WRO2026/LineFollower.cpp:368`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** line following
- **Traced path:** `followDistance(cm) -> waitUntilDistanceReached() -> update() -> !isOnline() && !_overshootMode -> LOST_LINE_ACTION switch -> return (skips the _distanceTargetActive gate at line 466)`

**What's wrong:** When a distance gate is armed (`_distanceTargetActive`) and the line goes offline on a
non-overshoot session, `update()` dispatches `LOST_LINE_ACTION` and unconditionally `return`s at line 368,
*before* reaching the distance-gate block at line 466. The encoder gate never advances or trips while the line
is lost. The wheels keep being commanded (STOP brakes, CRAWL creeps, COAST coasts) but `_distanceReached` is
never latched, so `waitUntilDistanceReached()` spins until `LINE_FOLLOW_WAIT_TIMEOUT_MS` (10 s) then returns
false. The overshoot path (322-341) deliberately falls through to the gate; the normal path does not. A real
gap, faded patch, or brief drift therefore stalls the segment for the full 10 s instead of dead-reckoning the
remaining distance.

**Options:**
- **A —** In the non-overshoot lost-line branch, evaluate the distance gate before returning (let the encoder gate trip even while the line is lost, mirroring the overshoot path). _(Lets dead-reckoning complete the segment through a gap; the gate is encoder-based and valid while lost.)_
- **B —** Keep the early return but shorten the lost-line timeout specifically when a distance gate is armed, so a gap fails fast rather than hanging 10 s. _(Less correct — still gives up instead of completing — but smaller.)_
- **C —** Leave as-is. _(Any line gap on a distance-gated follow hangs the mission 10 s then fails; not advisable.)_

**Recommendation:** A — fall through to the encoder gate while lost, matching the overshoot path's intent.

**Decision:** A — let the encoder distance-gate trip while the line is lost (dead-reckon through gaps; avoids the 10 s hang).

#### 18. Blocking follow helpers freeze sensor reads in the no-callback fallback under the shipped cache config

- **Location:** `WRO2026/LineFollower.cpp:789` (and the intersection-wait fallback at `:851`)
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** line following / intersection detection  *(merged: same defect at both fallback call sites, 789 & 851)*
- **Traced path:** `waitUntilDistanceReached()/waitUntilIntersection() -> _tickCallback == nullptr branch -> update() [+ Intersection.update()] without LineArray.update()`

**What's wrong:** `Config::LINE_ARRAY_TICK_CACHE_ENABLED` is `true` (Config.h:450). With caching on,
`WROLineArrayClass::update()` is the single refresh site and `getError()`/`isOnline()`/`getJunctionType()` are
pure cache readers. In the blocking helpers' fallback (no tick callback registered) the spin loops call only
`LineFollower.update()` (and `Intersection.update()`), never `LineArray.update()`. So the cached channels stay
frozen for the whole wait: the P+D loop, lost-line detection, and intersection edges all run on stale data, and
`waitUntilIntersection()` can never see a fresh edge. The header warns about this (LineFollower.h:545-548) but
many in-code comments still call the cache toggle "false (default)," which is stale and raises the odds a future
sketch ships without `setTickCallback`.

**Options:**
- **A —** Have the no-callback fallback loops call `LineArray.update()` themselves before `update()`. _(Makes the fallback self-sufficient regardless of cache config; the obvious fix.)_
- **B —** Make `setTickCallback` mandatory (assert/halt at `begin()` if no callback is registered before a blocking helper runs). _(Forces correct wiring; harsher, breaks bring-up sketches that intentionally skip it.)_
- **C —** Fix only the stale "false (default)" comments and keep the runtime behavior. _(Documentation-only; leaves the live freeze hazard.)_

**Recommendation:** A — refresh the array inside the fallback loop so the helpers work whether or not a callback is wired.

**Same defect, second call site:** `waitUntilIntersection()`'s fallback (LineFollower.cpp:851) ticks only
`update(); Intersection.update();` and likewise never calls `LineArray.update()`. With the cache enabled,
`getJunctionType()` is a pure cache reader, so the junction sample is frozen on entry, no rising edge ever fires,
and the wait can only time out (10 s) then brake — the mission stalls at the first junction even though the sensor
is physically over it. Fix A (refresh the array inside the fallback loop) covers both call sites.

**Decision:** A — refresh `LineArray` inside the no-callback fallback loop (insurance against a bring-up sketch that forgets `setTickCallback`).

#### 19. First post-reacquisition / post-freeze tick produces a derivative kick from resetting `_lastErr` to 0

- **Location:** `WRO2026/LineFollower.cpp:311`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** line following
- **Traced path:** `update() -> lost-line branch (_lastErr=0) OR freeze-exit branch (_lastErr=0) -> next online non-junction tick: dErr = err - _lastErr = err`

**What's wrong:** Both the lost-line branch (311) and the junction-freeze-exit branch (423) reset `_lastErr` to
0. On the next online non-junction tick the controller computes `dErr = err - 0 = err`, so `corr` includes
`LINE_KD*err` on top of `LINE_KP*err`. With `LINE_KD=4` and a reacquisition error near the edge (±3.5), this is
a one-tick derivative spike (clamped by `LINE_MAX_CORR` but still a hard correction) the instant the line
returns. Seeding `_lastErr` to the fresh `err` instead of 0 would zero the first `dErr`.

**Options:**
- **A —** Seed `_lastErr = err` on the first tick after a lost/freeze recovery (e.g. a "just-reacquired" flag that suppresses the derivative term for one tick). _(Eliminates the kick; smooth re-engagement.)_
- **B —** Zero only the integral/accumulator state on recovery and leave `_lastErr` to be naturally set by the first post-recovery read. _(Similar effect; depends on the exact state layout.)_
- **C —** Leave as-is. _(The spike is clamped; impact is a brief steering jerk on reacquisition — cosmetic but real.)_

**Recommendation:** A — seed `_lastErr` to the fresh error for one tick to kill the spurious derivative.

**Decision:** A — seed `_lastErr` to the fresh error for one tick after a lost/freeze recovery.

#### 20. Ramp-up progress is tracked from the LEFT wheel only while the gate / ramp-down use both wheels

- **Location:** `WRO2026/LineFollower.cpp:199`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** line following
- **Traced path:** `_applyEnableBuilder() -> _tShapeStartEnc = Drive.getEncL() ; update() ramp-up -> traveledCm = encDegToCm(abs(Drive.getEncL() - _tShapeStartEnc))`

**What's wrong:** The cruise ramp-up measures distance from the left wheel alone (lines 199-203), whereas the
ramp-down (254-256) and the distance gate (468-470) average both wheels. While the follower is steering hard
(large `corr`) or one wheel slips, the left wheel's degree delta is not forward travel, so the ramp-up consumes
its distance at a different point than the symmetric measure the gate uses. The two zones are tuned to meet via
a half-distance clamp from the symmetric `_distanceTargetCm`, but the ramp-up exit uses an asymmetric
single-wheel measure — they can drift on a curvy approach, producing a small cruise discontinuity at the seam.

**Options:**
- **A —** Measure ramp-up progress from the same both-wheel average the gate and ramp-down use (`(getEncL()+getEncR())/2`). _(Makes all three measures consistent; the clean fix.)_
- **B —** Leave the single-wheel ramp-up but widen the meeting tolerance so a small drift doesn't produce a visible step. _(Band-aid; doesn't address the asymmetry.)_
- **C —** Leave as-is. _(A minor speed bump at the ramp seam on curved approaches; low impact.)_

**Recommendation:** A — use the both-wheel average everywhere so the three measures agree.

**Decision:** A — use the both-wheel average for ramp-up (consistent with the gate + ramp-down).

#### 21. `deadbandSnap` floor makes CRAWL lost-line creep faster than the intended half-cruise

- **Location:** `WRO2026/LineFollower.cpp:361`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** shared math helpers
- **Traced path:** `update() lost-line branch -> CRAWL case -> deadbandSnap((int16_t)(cruise/2))`

**What's wrong:** The CRAWL recovery intends to creep at half cruise: `deadbandSnap((int16_t)(cruise/2))`. With
the SLOW preset (`SPEED_SLOW=30`), `cruise/2 = 15`, below `PWM_MIN_MAGNITUDE=20`, so `deadbandSnap` snaps it up
to 20 — the crawl runs at 67% of cruise, not 50%. With NORMAL (50) the half value 25 survives, so the gap only
appears at low presets. The comment "cruise/2 is already in-range" is false for SLOW. The helper is correct;
the call-site assumption is wrong.

**Options:**
- **A —** At low presets where `cruise/2 < PWM_MIN_MAGNITUDE`, either accept the floor as the deliberate minimum (update the comment) or pick a crawl speed explicitly. _(Pick one and make the comment match reality.)_
- **B —** Define a dedicated `CRAWL_PWM` constant in Config instead of deriving it from `cruise/2`. _(Removes the assumption; small Config addition.)_
- **C —** Leave as-is. _(The crawl is slightly faster than "half" at SLOW; cosmetic.)_

**Recommendation:** B — an explicit crawl constant removes the floor-vs-half ambiguity entirely.

**Decision:** B — add an explicit `CRAWL_PWM` constant in Config.

### Firmware — WROLineArray

#### 22. I2C bus failure returning `0x00` (not `0xFF`) is accepted as a valid all-white reading

- **Location:** `WRO2026/WROLineArray.cpp:639`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** line array sensor
- **Traced path:** `_refreshCache()/getError()/isOnline() -> _readBoardInternal() -> MatrixColor::getColor() -> i2cReadData() -> Wire.read()`

**What's wrong:** The transport-corruption guard only rejects CMYK bytes `> 99`. `MatrixColor::i2cReadData()`
issues a read with no transport-status check and returns `Wire.read()`, which returns 0 on a failed/empty
transaction. A bus failure surfacing as `0x00` passes the `>99` guard, unpacks to all-zero channels, and
`_readBoardInternal` returns true. After real calibration (LOW > 0), the clamp drives `norm` to 0, so the
failure manifests as a silent "lost line" rather than a detected read error. The method's own comment admits an
all-zero read is indistinguishable from off-line.

**Options:**
- **A —** Have the read path check the I2C transaction status (e.g. `Wire.endTransmission()` / `requestFrom` return) and report a distinct read-fault when the bus fails, separate from "all-white." _(Requires threading a status out of the vendor read; the only true fix.)_
- **B —** Treat a full all-zero board sample as suspect (e.g. require at least one non-zero channel or a sanity flag) and surface a read-fault rather than lost-line. _(Heuristic; can false-positive on a genuinely dark surface.)_
- **C —** Leave as-is. _(Bus dropouts degrade to lost-line behavior with no diagnostic; acceptable only if LOST_LINE_ACTION=STOP is deemed safe enough.)_

**Recommendation:** A if the vendor read can surface a status; otherwise document the limitation and rely on lost-line safety.

**Decision:** Defer — accept that bus dropouts degrade to lost-line; revisit only if a read-fault diagnostic is needed.

#### 23. CMYK transport-corruption guard only catches `0xFF`, not in-range corruption

- **Location:** `WRO2026/WROLineArray.cpp:656`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** 8-channel reflectance line array sensing
- **Traced path:** `getError()/isOnline()/getJunctionType() -> _refreshCache() -> _readBoardInternal() -> getColor(C/M/Y/K) raw Wire.read() -> guard if (bc>99 || ...) return false`

**What's wrong:** A fully failed `requestFrom` yields `read()==-1 -> 255`, caught by the `>99` guard. But any bus
glitch / stale-register read returning a value in [0..99] (e.g. `0x42==66`) is a structurally "valid" packed
byte (unpacks to channels 6,6) and sails through. Mid-range corruption is reported as a real, possibly strong
on-line reading, feeding `_computeError`/`_classifyJunction` a phantom line position or false junction instead
of falling into lost-line.

**Options:**
- **A —** Add an I2C transaction-status check so a failed/short read is rejected regardless of the byte value (same underlying fix as #22). _(Addresses both #22 and #23 at the source.)_
- **B —** Add a plausibility check across the full 8-channel sample (e.g. reject samples inconsistent with any real surface pattern). _(Heuristic, can't catch all in-range corruption.)_
- **C —** Leave as-is. _(In-range corruption is rarer than a full failure; residual risk of a phantom junction.)_

**Recommendation:** A — a transport-status check is the common fix for #22 and #23; do it once.

**Decision:** Defer — same as #22; lost-line safety deemed acceptable for now.

#### 24. `calibrate()` accepts a sweep with no usable contrast (LOW==HIGH) and emits no warning

- **Location:** `WRO2026/WROLineArray.cpp:318`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** line array sensor
- **Traced path:** `calibrate() -> _sweepOneBoardForCalibration() accumulates mnRow=min, mxRow=max over 5 s; post-sweep only checks validSamples[b]==0, never mxRow[c] > mnRow[c]`

**What's wrong:** Calibration records per-channel min as LOW and max as HIGH across a sweep meant to cover white
and the dark line. The only quality gate is `validSamples[b]==0`. If the operator sweeps only over white (or
only the line), a channel can end with `mn==mx`, printing `LOW==HIGH`. That row pastes into Config.h as a
dead/inverted channel (`LOW >= HIGH`), which `begin()`'s dead-channel warning flags and the read/classify paths
then silently skip — the channel is permanently disabled. `calibrate()` itself emits no warning for low/zero
dynamic range, so the operator only finds out at the next boot (or never).

**Options:**
- **A —** After the sweep, warn (and/or refuse to print a row) for any channel where `mxRow[c] - mnRow[c]` is below a minimum-contrast threshold. _(Catches the mistake at calibration time, when the operator can re-sweep; uses data already in hand.)_
- **B —** Print the dynamic range per channel so the operator can eyeball weak channels. _(Informational only; relies on the operator noticing.)_
- **C —** Leave as-is. _(Boot-time dead-channel warning is the only signal; easy to miss.)_

**Recommendation:** A — warn on insufficient contrast during `calibrate()`; the routine already has the min/max to do it.

**Decision:** Defer — rely on the boot-time dead-channel warning for now.

### Firmware — Intersection

#### 25. Two distinct junctions separated by fewer than `RELEASE_TICKS` NONE samples are merged into one edge

- **Location:** `WRO2026/Intersection.cpp:99`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** intersection detection
- **Traced path:** `Intersection.update() -> NONE gap shorter than INTERSECTION_RELEASE_TICKS -> _wasAtJunction stays true -> next valid streak suppressed by !_wasAtJunction guard`

**What's wrong:** Re-arming requires `INTERSECTION_RELEASE_TICKS` (2) consecutive NONE samples before
`_wasAtJunction` flips false (99-102). If the gap between two close junctions yields only one NONE tick (or the
between-marker region still classifies as a junction), `_noneStreak` never reaches the threshold,
`_wasAtJunction` stays true, and the second junction's valid streak is suppressed by the `!_wasAtJunction` guard
(line 92). `getEdgeCount()` advances once for two physical junctions, so PathTracker advances its FSM one node
short for the rest of the run. This is the intended debounce tradeoff, but at competition tick rates it can drop
a real junction.

**Options:**
- **A —** Lower `INTERSECTION_RELEASE_TICKS` to 1, or make it tick-rate-aware, so close junctions re-arm. _(Detects close pairs; raises the risk of double-counting a single fat junction — tune against the field.)_
- **B —** Detect junctions on a distance/encoder basis in addition to NONE-streak release, so spacing is measured in cm not ticks. _(More robust to tick rate and speed; more code.)_
- **C —** Leave as-is and ensure field junctions are spaced beyond the debounce window. _(Field-design constraint; legitimate if spacing is controllable.)_

**Recommendation:** C if junction spacing is a known field property; otherwise A with on-field tuning.

**Decision:** C / Defer — keep release=2, rely on field spacing. Verify against the real 2026 layout in P3.4; switch to A/B only if junctions sit within ~a few cm.

#### 26. Documented "shared single I2C sample" guarantee breaks when `LINE_ARRAY_TICK_CACHE_ENABLED` is false

- **Location:** `WRO2026/Intersection.cpp:59`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** intersection detection
- **Traced path:** `cooperativeTick() -> LineFollower.update() reads getJunctionType() (refreshes cache) -> Intersection.update() reads getJunctionType() again (refreshes a second time)`

**What's wrong:** Intersection.h:15-19 and LineFollower.h:21-23 assert both modules read the same I2C sample.
That only holds with the tick cache ON. When `LINE_ARRAY_TICK_CACHE_ENABLED` is false, each `getJunctionType()`
call runs `_refreshCache()` and issues its own I2C transaction. LineFollower samples it for its PID-freeze gate,
then Intersection samples it again — two physical reads milliseconds apart. The two can disagree (the controller
freezes for a tick Intersection classified NONE, or vice versa). The shipped Config has the toggle `true` so the
live build is consistent, but the header contract is violated for the documented `false` configuration that the
headers call the "default."

**Options:**
- **A —** Update the header docs and the in-code comments to state the shared-sample guarantee holds only with the cache enabled, and that the shipped default is `true`. _(Documentation fix; matches reality.)_
- **B —** Make a single shared sample explicit (one `LineArray.update()` per tick feeding both consumers) regardless of the cache toggle, so the guarantee always holds. _(Removes the footgun for the `false` config; small refactor of the tick ordering.)_
- **C —** Remove the `false` configuration path entirely if it is no longer supported. _(Simplifies, but drops a documented option.)_

**Recommendation:** A (with B if the `false` mode must stay supported) — at minimum correct the stale "default = false" claim.

**Decision:** A — correct the stale "default = false" docs (shipped cache is true).

### Firmware — EventTrace

#### 27. Intersection events are documented and primed but never emitted (no Intersection poll in `update()`)

- **Location:** `WRO2026/EventTrace.cpp:72` (and the missing "poll 2" at `:84`)
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** intersection detection / event trace / OLED telemetry  *(merged: found under both labels)*
- **Traced path:** `begin() caches _prevIntEdgeCount = Intersection.getEdgeCount() -> update() polls LineFollower (poll 1) and Drive (poll 3) only; no poll comparing getEdgeCount() against _prevIntEdgeCount`

**What's wrong:** `EventTrace.begin()` (EventTrace.cpp:36) primes `_prevIntEdgeCount = Intersection.getEdgeCount()`,
and the header (EventTrace.h:9-10, 16-22, 137-144) plus Intersection.h:138 say EventTrace is the passive observer
that fires whenever `getEdgeCount()` advances. But `update()` (72-107) only contains "Source poll 1 — LineFollower"
and "Source poll 3 — Drive" plus the heartbeat. There is no "poll 2": `getEdgeCount()` is read once at `begin()`
and `_prevIntEdgeCount` is never compared again, so no intersection event ever reaches the ring buffer or OLED —
the single consumer designed to surface junctions does nothing, and `_prevIntEdgeCount` is dead state.

**Options:**
- **A —** Add the missing poll: compare `Intersection.getEdgeCount()` against `_prevIntEdgeCount`, push a junction event when it advances, update the cache. _(Restores the documented behavior; the obvious fix.)_
- **B —** Push the event from inside Intersection's rising-edge path instead of polling from EventTrace. _(Couples Intersection to EventTrace; breaks the passive-observer design the headers describe.)_
- **C —** Remove the documentation and the dead `_prevIntEdgeCount` priming and accept that junctions are not traced. _(Honest but loses a useful debug signal the design promised.)_

**Recommendation:** A — implement the poll the header already documents; it is a few lines mirroring poll 1/3.

**Decision:** C — keep current behavior (junctions not traced); fix the docs and drop the dead `_prevIntEdgeCount` priming to match.

#### 28. Heartbeat reads IMU yaw live every period even when idle, contradicting the "never spins on I/O" contract

- **Location:** `WRO2026/EventTrace.cpp:124`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** event trace / OLED telemetry
- **Traced path:** `update() -> _emitHeartbeat() -> Localization.getHeadingDeg() -> MiniR4.Motion.getEuler(Yaw) every MISSION_HEARTBEAT_MS`

**What's wrong:** `_emitHeartbeat()` calls `Localization.getHeadingDeg()` unconditionally each heartbeat, which
performs a live `MiniR4.Motion.getEuler(Yaw)` read when zeroed. The EventTrace header (lines 26-27) advertises
`update()` as non-blocking and never spinning on I/O, yet a synchronous IMU/I2C read is issued from the telemetry
path. If `getEuler()` is a blocking lower-MCU transaction, the heartbeat injects periodic bus latency into the
main loop every 500 ms post-zero, from a module whose contract claims to do no I/O.

**Options:**
- **A —** Have the heartbeat read a cached heading (last value PathTracker/Localization already fetched) instead of issuing its own IMU read. _(Removes the bus round-trip from telemetry; needs a cached-heading accessor.)_
- **B —** Drop heading from the heartbeat line, or print it only when navigation already read it this tick. _(Simplest; slightly less informative heartbeat.)_
- **C —** Leave as-is after confirming `getEuler` is cheap/non-blocking. _(Legitimate only if the read is confirmed non-blocking — see Review-manually item on `getEuler`.)_

**Recommendation:** A — serve the heartbeat from a cached heading so the telemetry path keeps its no-I/O contract.

**Decision:** A — serve the heartbeat from a cached heading (no bus I/O in telemetry). Possible link to the "LRS" symptom — under investigation; may raise priority.

### Firmware — Localization

#### 29. `zeroHeadingFromCurrent` latches a wrong frame when the IMU read silently times out

- **Location:** `WRO2026/Localization.cpp:29`
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** localization
- **Traced path:** `zeroHeadingFromCurrent() -> MiniR4.Motion.getEuler(Yaw) -> MMLower::GetIMUEuler() returns timeout without writing yaw -> getEuler() returns 0.0 (result ignored) -> _yawOffsetDeg = 0.0 - START_HEADING_DEG, _zeroed=true, returns true`

**What's wrong:** `zeroHeadingFromCurrent()` treats `getEuler(Yaw)` as always valid. The underlying
`MMLower::GetIMUEuler` (MMLower.cpp:1120) returns early on a comms timeout *without writing the yaw out-param*,
and `MiniR4Motion::getEuler` initializes its local yaw to 0 and discards the RESULT enum — so a transient STM32
timeout makes `getEuler(Yaw)` return `0.0` rather than signal failure. `zeroHeadingFromCurrent` then computes
`_yawOffsetDeg = 0.0 - START_HEADING_DEG`, latches `_zeroed=true`, and returns true. Its docstring claims it
returns false on un-readiness, but there is no readiness/error check, and the `.ino` BTN_UP handler treats the
true return as success — arming the mission with a corrupt world-frame offset and every subsequent `turnToYaw`
off by the IMU's boot yaw.

**Options:**
- **A —** Read the IMU through an API that surfaces the RESULT (or add a wrapper that checks `GetIMUEuler`'s return), and return false from `zeroHeadingFromCurrent` on a non-OK read so the `.ino` retry path engages. _(Closes the silent-success hole; the BTN_UP handler already handles a false return.)_
- **B —** Take several reads and require agreement/non-zero variance before accepting the zero. _(Robust against a single bad sample; more code, still no explicit error surface.)_
- **C —** Leave as-is. _(A single timed-out zero corrupts the whole run's heading frame with no diagnostic; risky for a one-shot arm action.)_

**Recommendation:** A — surface the read result and return false on failure; the retry path is already wired.

**Decision:** A — surface the IMU read result; return false on a bad read so the wired retry fires instead of arming with a corrupt frame.

#### 30. `getHeadingDeg` returns a garbage heading on a transient IMU read failure instead of flagging staleness

- **Location:** `WRO2026/Localization.cpp:56`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** localization
- **Traced path:** `PathTracker::_onPlanNextHop() -> Localization.getHeadingDeg() -> getEuler(Yaw) times out, returns 0.0 -> _normalize(0.0 - _yawOffsetDeg) -> used as currentWorld for the turn-vs-skip decision`

**What's wrong:** `getHeadingDeg()` reads the IMU live every call with no validity check. As in #29, a transient
`GetIMUEuler` timeout yields a silent `0.0`. When zeroed, `getHeadingDeg` then returns
`_normalize(0.0 - _yawOffsetDeg)` — wrong by up to a full revolution for that one read. PathTracker consumes
this live value (PathTracker.cpp:690) to compute `delta = bodyTarget - currentWorld` and decide whether/how far
to turn, so one dropped frame can flip the turn decision or dispatch a spurious large turn.

**Options:**
- **A —** Surface the read result; on a non-OK read, return the last good heading (and/or a staleness flag callers can check) rather than `0.0`-derived garbage. _(Prevents a single dropped frame from flipping a turn decision.)_
- **B —** Have PathTracker filter/reject implausible single-tick heading jumps before acting. _(Defense at the consumer; doesn't fix other callers.)_
- **C —** Leave as-is. _(Low per-tick probability, but the failure dispatches a spurious turn mid-run.)_

**Recommendation:** A — return the last good heading on a failed read; fixes this and feeds #28's cache.

**Decision:** A — return the last good heading on a non-OK read (also provides the cache #28 uses).

#### 31. `onIntersection` / `onDeadReckonedAdvance` accept any `uint8_t` node id with no range check

- **Location:** `WRO2026/Localization.cpp:72`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** localization
- **Traced path:** `PathTracker advance -> Localization.onIntersection(_nextHop)/onDeadReckonedAdvance(_nextHop) -> _currentNode = advanceTo (no validation) -> FieldMap::getEdge(_currentNode, ...) indexes EDGES[_currentNode][...]`

**What's wrong:** Both writers (lines 72, 77) assign `advanceTo` to `_currentNode` with zero validation,
trusting the caller. `_currentNode` is the single source of truth FieldMap then uses as an unchecked array index
(`EDGES[from][to]`, `NODES[id]`, `NEXT_HOP[from][goal]`). If a caller ever passed `NODE_NONE` (0xFF) or any
`id >= N_NODES`, the next FieldMap lookup reads out of bounds. The codebase already knows this hazard
(EventTrace.cpp:119 guards `node < N_NODES`), but these setters do not. Today `_nextHop` is guarded in
`_onPlanNextHop` before reaching the advance states, so this is a latent defense-in-depth gap, not a live crash.

**Options:**
- **A —** Validate `advanceTo` in both setters (reject `NODE_NONE` / `>= N_NODES`, log a fault) before writing `_currentNode`. _(Cheap defense-in-depth at the single-writer boundary.)_
- **B —** Add the bounds check inside FieldMap accessors instead (see #33), covering all callers at once. _(Broader; one place to guard.)_
- **C —** Leave as-is, relying on every caller validating upstream. _(Works today but is one refactor away from an OOB read.)_

**Recommendation:** B — guard in FieldMap (finding #33) so every consumer including these setters is covered.

**Decision:** B — covered by the FieldMap accessor guard (#33).

### Firmware — PathTracker

#### 32. Stale `_passThrough` survives the yaw-vs-peek divergence turn and corrupts the next BLIND dispatch

- **Location:** `WRO2026/PathTracker.cpp:803`
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** path tracking / autonomous graph navigation
- **Traced path:** `LINE_FOLLOW sets _passThrough=true -> LINE_OVERSHOOT (controller stays enabled, advance) -> _onPlanNextHop() delta>NAV_TURN_TOLERANCE_DEG (IMU drift vs edge-heading peek) -> disable follower, dispatch turnToYaw, _state=TURNING (does NOT reset _passThrough) -> _dispatchEdgeAfterTurn() BLIND branch reads stale _passThrough==true`

**What's wrong:** When the LINE_FOLLOW peek classifies the next hop as pass-through (sets `_passThrough=true`,
leaves the follower running with `brakeOnGate=false`) but `_onPlanNextHop()` then decides a pre-edge turn *is*
required (the divergence guard at 778-795 fires because live IMU heading disagrees with the FieldMap edge
headings the peek used), the turn branch disables the follower and dispatches `turnToYaw` but never clears
`_passThrough`. After the turn (chassis stopped), `_dispatchEdgeAfterTurn()` runs the BLIND pass-through branch
(937-941): it reads `startSpeed = presetToPwm(LineFollower.getPreset())` (cruise, e.g. 50) instead of the
soft-start floor (`TSHAPE_START_SPEED=20`). The blind move jumps straight to cruise from a dead stop with no
ramp, jolting the chassis. LINE/PROJECTION dispatches are unaffected; the defect is scoped to a BLIND next-edge
after a divergence-guard turn. *(See also Review-manually item at PathTracker.cpp:792 — same defect, uncertain bucket.)*

**Options:**
- **A —** Reset `_passThrough = false` when the divergence guard decides a turn is required (before dispatching `turnToYaw`). _(Directly fixes the stale-flag read; the chassis is stopped after the turn so the blind move should soft-start.)_
- **B —** In `_dispatchEdgeAfterTurn()` BLIND branch, derive `startSpeed` from the actual chassis state (it is at rest after a TURN_BRAKE turn) rather than trusting `_passThrough`. _(More robust — the start speed reflects reality regardless of the flag — but a larger change to the dispatch logic.)_
- **C —** Leave as-is. _(Requires a specific drift-past-tolerance condition; the symptom is a one-time jolt, not a wrong route.)_

**Recommendation:** A — clear `_passThrough` on the turn-required branch; it is the precise, minimal fix.

**Decision:** A — clear `_passThrough` on the turn-required branch (covers #90).

### Firmware — FieldMap / DirectionPolicy / WROMath

#### 33. FieldMap `getNextHop`/`getEdge`/`getNode` perform no bounds checking on node indices

- **Location:** `WRO2026/FieldMap.h:37`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** field graph data and route table
- **Traced path:** `FieldMap::getNextHop(from,goal) -> Routes::NEXT_HOP[from][goal]; getEdge(from,to) -> EDGES[from][to]; getNode(id) -> NODES[id]`

**What's wrong:** `getNextHop` (39), `getEdge` (47), `getEdgeType` (55), `getNode` (63), `getNodeType` (71) all
index the fixed-size `Routes::*` arrays with the raw `uint8_t` arguments and zero validation. Any index
`>= N_NODES` (including `NODE_NONE` 0xFF) reads out of bounds. Only `findNodeByName` defends itself. The
codebase already treats this as a hazard elsewhere (EventTrace.cpp:119 guards before indexing `NODES`; PathTracker
and Mission pre-validate goal ids), so FieldMap relies entirely on every caller validating first. Today all live
call sites happen to validate upstream, so no OOB read fires — it is a latent contract gap.

**Options:**
- **A —** Add a bounds check to each accessor; on an out-of-range index return a safe sentinel (e.g. `NODE_NONE` / a zeroed `EdgeInfo`) and optionally log. _(Defends all callers including #31's setters; cheap, header-only.)_
- **B —** Add `assert`/debug-only guards so release builds stay branch-free but debug builds catch the OOB. _(No release-path cost; only catches in debug, which this firmware rarely runs.)_
- **C —** Leave as-is, documenting "callers must validate." _(Status quo; one refactor from a silent OOB read on the auto-generated tables.)_

**Recommendation:** A — guard at the accessor boundary so the single source of indexing is safe for every caller.

**Decision:** A — add bounds checks to the FieldMap accessors (cheap; also covers #31).

#### 34. Cross-language TC18 "parity" is only exercised against the Python implementation

- **Location:** `WRO2026/DirectionPolicy.h:36`
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** bidirectional direction resolution
- **Traced path:** `DirectionPolicy.h docstring "TC18 enforces it" -> test_runtime_sim.py (TC1/TC2 call only Python resolve_direction) ; test_path_tracker_bidirectional.cpp (calls C++ resolveDirection)`

**What's wrong:** Both `DirectionPolicy.h` and `runtime.py` claim TC18 keeps the two implementations
"step-for-step identical." In practice the C++ test calls the shared inline `resolveDirection` (good — no C++
replica drift) and the Python tests call the Python `resolve_direction`. The two wrap routines are independently
written (C++ unbounded while-loop vs Python modulo in `compiler.normalize_turn`). No test feeds the same input
set through *both* and asserts equal output; the TC14a/b "parity" tests only regex-grep the header for constant
values (threshold 90.0, AUTO flag), not the algorithm. A divergence in the wrap or the float/double threshold
would pass CI.

**Options:**
- **A —** Add a cross-language golden test: run a shared input vector through the Python `resolve_direction` and assert it matches recorded C++ `resolveDirection` outputs (or expose the C++ via a tiny harness). _(Actually enforces the parity the docs claim.)_
- **B —** Generate the threshold/constants from a single source so the two can't drift on values (already partly true), and document that the *algorithm* parity is by-inspection only. _(Cheaper; weaker guarantee.)_
- **C —** Leave as-is and soften the "TC18 enforces parity" docstrings to "by inspection." _(Honest documentation fix; no new guard.)_

**Recommendation:** A — a shared-vector golden is the only thing that makes the parity claim true.

**Decision:** A — add a shared-vector parity golden so a C++/Python divergence fails CI.

#### 35. `cmToEncDeg` silently wraps on the positive side — `uint16_t` cast has no upper-bound guard

- **Location:** `WRO2026/WROMath.h:136`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** shared math helpers
- **Traced path:** `cmToEncDeg(cm) -> revs = (cm*10)/(PI*WHEEL_DIAM_MM) -> return (uint16_t)(revs*360.0f)`

**What's wrong:** `cmToEncDeg` guards the negative input (clamps `cm<0` to 0, with a comment that a negative
float→uint16_t is UB) but does not guard the result's upper bound. With `WHEEL_DIAM_MM=60`, ~19.1 enc-deg/cm,
the `uint16_t` saturates at `cm ≈ 3431` cm (~34 m); beyond that the float→uint16_t conversion is UB and in
practice wraps mod 65536, turning a long distance into a short one. The Drive executors and `_execProfileMove`
feed this value straight to `MoveDegs`/profile state, storing it into a wider `int32_t` only *after* the 16-bit
truncation. Not reachable on the ~2 m WRO field with a single move — a latent ceiling.

**Options:**
- **A —** Clamp the result to `UINT16_MAX` (mirroring the negative-side clamp) and optionally log when clamping. _(Symmetric with the existing negative guard; trivial.)_
- **B —** Return a wider type (`uint32_t`) so the ceiling matches the downstream `int32_t` storage. _(Removes the truncation entirely; touches call sites.)_
- **C —** Leave as-is. _(Unreachable on this field; only bites a tiny-wheel config or a >34 m move.)_

**Recommendation:** A — add the upper clamp for symmetry; it is a one-liner and removes the silent wrap.

**Decision:** A — clamp the upper bound (one-liner, mirrors the existing negative guard).

### Offline — build_routes (CLI orchestrator)

#### 36. `ValueError` from the pivot out-degree guard crashes the CLI with no clean exit code

- **Location:** `tools/path_planner/build_routes.py:189`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** offline path planner pipeline
- **Traced path:** `_run() -> compute_all_pairs_next_hop() -> apply_pivot_compensations() raises ValueError; _run only wraps validate() in try/except (FieldDataError)`

**What's wrong:** `apply_pivot_compensations` raises `ValueError` when a pivot-tagged edge's destination has zero or
more than one forward out-edge. The validator deliberately defers this check (validator.py:280-283 say the
cross-edge geometry check is "deferred to planner.py (T5)"), so a fully valid `field_data` can pass `validate()`
and still trip the ValueError. In `_run()`, `compute_all_pairs_next_hop()` (step 3) is called outside any
try/except; only `validate()` (step 1) is guarded, and the only broad `Exception` catch is around `render_map()`
(step 6). The ValueError propagates as an uncaught traceback — no clean ERROR message, not the documented exit
code 2 (data error) — and routes.h is left unwritten with no controlled exit.

**Options:**
- **A —** Catch the pivot guard's failure in `_run()` (or raise it as a `FieldDataError`) and exit 2 with a readable "pivot compensation requires exactly one out-edge" message. _(Brings it into the documented data-error path.)_
- **B —** Move the out-degree check into the validator so it fails at step 1 with a clean diagnostic. _(Earliest, clearest surface; the validator already plans to own this — "deferred to T5".)_
- **C —** Leave as-is. _(A natural Y/T junction with a pivot edge dies with a raw traceback instead of a readable error.)_

**Recommendation:** B — promote the out-degree check into the validator so the failure is a clean FieldDataError at validation time.

**Decision:** B — move the pivot out-degree check into the validator (clean FieldDataError instead of a raw traceback).

### Offline — graph_builder

#### 37. Obstacle-clipped LINE_SEGMENTS edges are dropped silently while BLIND/EDGES warn

- **Location:** `tools/path_planner/graph_builder.py:363`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** offline path planner pipeline / field graph builder  *(merged: found under both labels)*
- **Traced path:** `build_graph() -> for u,v in LINE_SEGMENTS: if edge_clips_obstacle(...): continue [no warnings.warn]`

**What's wrong:** When a LINE_SEGMENTS edge is clipped by an inflated obstacle it is dropped with a bare
`continue` and no `UserWarning`, whereas the identical drop for BLIND_EDGES (line 371/377) and EDGES (line
427/433) emits a `UserWarning` that `_run()` forwards to stderr as a `WARN:` line. So a primary line route
vanishing from the graph is invisible. With the current `field_data.json` this is live: the only LINE_SEGMENTS
entry (Inter1=(67.9,36.9)→Inter2=(240.3,36.9)) passes through the single obstacle inflated by
`ROBOT_RADIUS_CM=15` (inflated box x[112.95,159.76] y[9.0,41.857]; the y=36.9 line crosses it), so the line edge
is removed and Inter1/Inter2 are left connected only via the longer "finial" blind detour — with zero feedback.
The only indirect signal is the much later "Node has no incident edges" warning, and only if the drop left a
node fully isolated.

**Options:**
- **A —** Emit a `warnings.warn(...)` on the LINE_SEGMENTS clip, matching the BLIND/EDGES branches. _(Restores symmetry; the operator sees the drop. One line.)_
- **B —** Make any obstacle-clipped edge a hard error (fail the build) rather than a warning. _(Stronger — a dropped route is usually a mistake — but changes pipeline behavior for all three edge types.)_
- **C —** Leave as-is. _(Silent line-route loss is the hardest planner bug to debug; not advisable given the live field already triggers it.)_

**Recommendation:** A — warn on the LINE drop exactly like BLIND/EDGES; this is the headline offline bug since it fires on the committed field.

**Decision:** A — warn on a clipped LINE edge like BLIND/EDGES do. Fix before the real-field encode (P3.5/P3.6).

#### 38. An EDGES entry duplicating a LINE/BLIND pair silently overwrites edge type and geometry

- **Location:** `tools/path_planner/graph_builder.py:434`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** field graph builder with obstacle inflation
- **Traced path:** `LINE_SEGMENTS emits (A,B) type=LINE -> EDGES loop emits same (A,B) via _emit_directed_pair(..., edge_type, options) -> g.add_edge overwrites type/distance_cm/heading_deg on both directed edges`

**What's wrong:** The validator's duplicate-pair cross-check (validator.py:376-384) only rejects a pair appearing
in *both* LINE_SEGMENTS and BLIND_EDGES. Neither the validator nor `build_graph` checks whether an EDGES entry
collides with an existing LINE/BLIND pair. Because `build_graph` emits LINE then BLIND then EDGES (360-434) and
networkx `add_edge` replaces attributes on an existing edge, an EDGES entry like `{type: EDGE_BLIND, u:'A', v:'B'}`
shadowing a LINE pair silently flips both directed edges' `type` to BLIND (changing the Dijkstra cost via
`cost_blind_multiplier`) and overwrites their `distance_cm`/`heading_deg` — with no warning. Exactly the silent
corruption the LINE/BLIND guard was written to prevent, bypassed via EDGES. (See also #47, the validator-side
view of the same gap.)

**Options:**
- **A —** Add a cross-check (validator or build_graph) that rejects/warns when an EDGES entry's pair already exists in LINE_SEGMENTS or BLIND_EDGES with a conflicting type. _(Closes the bypass at the right layer; preferably in the validator for a clean FieldDataError.)_
- **B —** Make EDGES strictly an *override* layer: require the type to match the existing pair, allowing only attribute (distance/heading/pivot) overrides. _(Encodes the likely intent; needs a clear override semantics.)_
- **C —** Leave as-is. _(Routing cost and primitive silently depend on emission order; hard-to-debug data corruption.)_

**Recommendation:** A — extend the validator's anti-ambiguity rule to cover EDGES vs LINE/BLIND.

**Decision:** A — validator cross-check rejecting an EDGES pair that conflicts with an existing LINE/BLIND pair (one fix with #47).

#### 39. `edge_clips_obstacle` treats boundary-touching segments as clipping (intersects vs crosses)

- **Location:** `tools/path_planner/graph_builder.py:166`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** field graph builder with obstacle inflation
- **Traced path:** `build_graph() -> edge_clips_obstacle() -> LineString.intersects(poly)`

**What's wrong:** `edge_clips_obstacle` uses `line.intersects(poly)`, which returns True for a mere boundary touch
(tangent / shared endpoint), not just a true interior crossing. After inflation by `ROBOT_RADIUS_CM`, a node
placed exactly one robot-radius from a wall — the *intended* minimal clearance — will have its incident segment
graze the inflated polygon edge and be dropped. A `crosses`/interior-only test would not. Combined with #37
(silent LINE drops), an edge that merely kisses the clearance boundary is removed without warning.

**Options:**
- **A —** Use an interior-crossing test (e.g. `line.crosses(poly)` or `intersection(...).length > eps`) so a tangent/grazing touch at the legal clearance does not count as clipping. _(Matches the physical "just clears" intent; small change, verify against existing fixtures.)_
- **B —** Shrink the inflation by a tiny epsilon so a node placed exactly at clearance is robustly inside the legal region. _(Hacky; trades one float edge case for another.)_
- **C —** Leave as-is and require nodes to sit strictly beyond clearance. _(Field-authoring constraint; legitimate but surprising at minimal clearance.)_

**Recommendation:** A — an interior-crossing predicate is the correct geometry for "the robot would clip the obstacle."

**Decision:** A — use an interior-crossing test so an edge at the minimum legal clearance isn't dropped.

#### 40. Reverse directed edge inherits no directional override; a one-sided EDGES `direction`/`pivot` is silently skipped when the planner traverses v→u

- **Location:** `tools/path_planner/graph_builder.py:302`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** field graph builder with obstacle inflation
- **Traced path:** `EDGES entry with direction/pivot/pre_turn_mode -> _emit_directed_pair -> reverse g.add_edge gets only **bidirectional`

**What's wrong:** By design (Codex P1 invariant, 179-186), `pre_turn_mode`/`pivot`/`direction_override` apply to
the forward (u→v) edge only; the reverse (v→u) edge gets only the symmetric `primitive_mode`. This is documented
and intentional, but it is a footgun: adding one EDGES entry with `direction='rear'` affects only the u→v hop. If
NEXT_HOP routes through v→u (common, since edges are bidirectional and Dijkstra picks the shorter direction), the
override silently does not apply and the reverse hop falls back to `PIPELINE_CONFIG.direction`. No warning that
the override is single-directional.

**Options:**
- **A —** Warn at build time when an edge carries a directional override but the planner's chosen NEXT_HOP traverses the un-overridden direction. _(Surfaces the skipped intent; needs the planner result to cross-check.)_
- **B —** Mirror the directional override onto the reverse edge with the geometrically-appropriate transform (rear↔front, pivot mirrored). _(Makes the override symmetric; only correct if mirroring semantics are well-defined.)_
- **C —** Leave as-is and document the single-direction behavior prominently at the EDGES authoring surface. _(Documentation-only; matches the deliberate invariant.)_

**Recommendation:** C (document loudly) with A if a build-time cross-check is feasible — do not silently mirror without clear semantics.

**Decision:** C — document the single-direction override loudly at the EDGES authoring surface (intentional invariant, but a footgun).

#### 41. `_resolve_override` crashes on string `primitive_mode` tags (unvalidated fixtures)

- **Location:** `tools/path_planner/sim/runtime.py:290`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** offline path planner pipeline
- **Traced path:** `compile_path()/PathTrackerSim.go_to() -> resolve_line_mode(edge) -> _resolve_override() -> enum_cls(override) where override is a str`

**What's wrong:** `_resolve_override` handles `None`, an enum instance, or an int (`enum_cls(int)`), but a graph
edge carrying `primitive_mode` as a bare string name (e.g. `'NORMAL'`) hits `enum_cls('NORMAL')`. The mode enums
are `IntEnum`, so `LineMode('NORMAL')` raises `ValueError` (only `LineMode['NORMAL']` works by name).
`graph_builder` forwards `primitive_mode` verbatim, so a string survives onto the edge. The strict validator
requires an enum instance (validator.py:222), so the fully-validated pipeline never hits this — but tests and any
code building a graph via `_emit_directed_pair(options={'primitive_mode': 'NORMAL'})` (test_planner.py:229)
bypass validation and crash here. `header_writer._override_enum_value` correctly handles string tags via
`enum_cls[mode_tag]`, so the two consumers disagree on the accepted type.

**Options:**
- **A —** Make `_resolve_override` accept string names via `enum_cls[name]` (matching `header_writer`), falling back to `enum_cls(int)` for ints. _(Aligns the two consumers; small.)_
- **B —** Normalize `primitive_mode` to an enum at graph-build time so no string ever reaches either consumer. _(Single source of truth; touches `graph_builder`.)_
- **C —** Leave as-is and require fixtures to pass enum instances. _(Status quo; the two consumers stay inconsistent.)_

**Recommendation:** A — accept the same string form `header_writer` already accepts, so sim and emitter agree.

**Decision:** A — accept string mode names (match `header_writer`) so sim and emitter agree.

### Offline — planner

#### 42. Pivot compensation only changes Dijkstra cost; the shortened distance never reaches the emitted drive distances

- **Location:** `tools/path_planner/planner.py:320`
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** offline path planner pipeline / all-pairs Dijkstra planner  *(merged: found under both labels)*
- **Traced path:** `compute_all_pairs_next_hop() -> _weighted_copy() + apply_pivot_compensations() [mutates private weighted copy only] -> only next_hop strings returned, weighted discarded -> emit_routes(graph=ORIGINAL) -> compiler reads original edge['distance_cm']`

**What's wrong:** `apply_pivot_compensations()` shortens the following edge's `distance_cm`/heading only on the
local `weighted` copy used to compute NEXT_HOP (planner.py:320, 237). `compute_all_pairs_next_hop` returns just
the next_hop string table (line 353); the compensated `weighted` copy is discarded. `build_routes` then passes
the *original, uncompensated* graph to `emit_routes`/compiler, so the compiler emits the un-shortened
`distance_cm` into routes.cpp's EDGES. Net: pivot compensation influences only which route Dijkstra prefers; the
physical distance the robot drives is never corrected. The planner/compensation docstrings claim the reported
costs and the driven displacement reflect the compensated geometry — false. The robot overshoots by the chord's
forward component on every pivot landing.

**Options:**
- **A —** Carry the compensated distances through to emission: return the compensated graph (or apply the compensation to the graph that `emit_routes` consumes) so routes.cpp's EDGES carry the shortened distance. _(Makes the compensation actually affect motion; verify it doesn't double-affect cost.)_
- **B —** Apply pivot compensation directly to the original graph before both Dijkstra and emission. _(Single graph, no copy/discard mismatch; simplest if cost and distance should both reflect it.)_
- **C —** Drop pivot compensation entirely and document that pivots are not distance-corrected. _(Honest if the feature is not needed yet; loses the landing accuracy it was meant to provide.)_

**Recommendation:** B — compensate the one graph used for both cost and emission so the driven distance matches the documented intent.

**Decision:** B — apply pivot compensation to the single graph used for both Dijkstra cost and emission (only matters if the real field uses pivots).

#### 43. Entry point does not validate `cost_blind_multiplier`; zero/negative values break Dijkstra correctness

- **Location:** `tools/path_planner/planner.py:281`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** all-pairs Dijkstra planner
- **Traced path:** `compute_all_pairs_next_hop(graph, cost_blind_multiplier) -> _weighted_copy() -> cost = distance_cm * multiplier -> nx.single_source_dijkstra(weight='cost')`

**What's wrong:** `compute_all_pairs_next_hop` applies `cost_blind_multiplier` directly (270, 271, 325, 326) with
no positivity check. The docstring says "Must be positive," but that is only enforced by validator.py:424 inside
`build_graph`, a separate call. A multiplier of `0.0` makes every BLIND edge free (Dijkstra routes through
arbitrarily many blind hops); a negative multiplier yields negative edge costs, violating Dijkstra's
non-negative-weight assumption and producing wrong next-hop choices. The test suite already calls the planner
directly with literal multipliers (test_planner.py:66-69), so the entry point is exercised outside the validated
pipeline.

**Options:**
- **A —** Add a `cost_blind_multiplier > 0` check at the top of `compute_all_pairs_next_hop` (raise ValueError). _(Defends every direct caller; matches the docstring contract.)_
- **B —** Move/duplicate the validator's positivity check so any planner entry enforces it. _(Same effect; consolidates the rule.)_
- **C —** Leave as-is, relying on the validator. _(Direct callers/tests can silently get a wrong table.)_

**Recommendation:** A — guard at the public entry so direct callers can't produce a silently-wrong routing table.

**Decision:** A — guard `cost_blind_multiplier > 0` at the planner entry.

### Offline — compiler

#### 44. `final_heading_after` ignores REAR direction, returns wrong heading for reverse hops

- **Location:** `tools/path_planner/compiler.py:79`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** path compiler
- **Traced path:** `final_heading_after() -> current = graph.edges[u, v]["heading_deg"] for every hop (no direction resolution)`

**What's wrong:** `final_heading_after` computes the final heading as the raw forward edge heading of the last hop
(line 79), with no direction resolution. But `compile_path`, when a hop resolves to `Direction.REAR`, leaves the
chassis body heading at `(edge_heading + 180) % 360` (compiler.py:194-195, 208). So whenever the final hop is
REAR, `final_heading_after` disagrees with the actual end heading by 180°. Its docstring claims it returns "the
heading the chassis would have at path[-1]" — false for REAR hops. It never consults `resolve_direction` /
`PIPELINE_CONFIG.direction` the way `compile_path` does.

**Options:**
- **A —** Resolve the last hop's direction (reuse `resolve_direction` / `PIPELINE_CONFIG`) and add 180° when it is REAR. _(Makes the returned pose correct for all hops; mirrors `compile_path`'s logic.)_
- **B —** Have `final_heading_after` reuse `compile_path`'s computed end state directly rather than recomputing from the raw edge. _(Single source of truth; avoids re-deriving direction.)_
- **C —** Leave as-is and document that the result is the forward edge heading, not the chassis heading. _(Mislabels a function whose name promises the chassis heading; not advisable.)_

**Recommendation:** B — derive the end pose from `compile_path`'s resolution so the two can never disagree.

**Decision:** B — derive the end pose from `compile_path`'s direction resolution so it can't disagree with the driven heading.

#### 45. Primitive type alias (2-tuple) is violated by the 3- and 4-tuple reverse primitives

- **Location:** `tools/path_planner/compiler.py:56`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** path primitive compiler
- **Traced path:** `compile_path REAR branch appends (ACT_LINE_REVERSE, v, mode) [3-tuple] and (ACT_*_REVERSE, v, distance, mode) [4-tuple] into List[Primitive] where Primitive = Tuple[str, Optional[float]]`

**What's wrong:** The module declares `Primitive = Tuple[str, Optional[float]]` and annotates `compile_path` as
`List[Primitive]`. The REAR branches emit 3-tuples `(ACT_LINE_REVERSE, v, line_mode)` and 4-tuples
`(ACT_PROJECTION_REVERSE / ACT_BLIND_REVERSE, v, distance_cm, mode)` whose 2nd element is a node name (str) and
which carry extra fields. The annotation is therefore wrong, and the second slot is sometimes a str/None rather
than `Optional[float]`. Python doesn't enforce this at runtime, but any typed consumer indexing `prim[1]`
expecting a float gets a node name or an arity mismatch on reverse primitives.

**Options:**
- **A —** Widen the type alias to a `Union` of the actual tuple shapes (or a small dataclass per primitive). _(Makes the published contract match reality; best for typed consumers.)_
- **B —** Normalize all primitives to a single fixed-arity shape (pad with None) so the 2-tuple-ish contract holds uniformly. _(Uniform shape; changes the emitted data layout and downstream readers.)_
- **C —** Leave as-is. _(Works at runtime today; misleads static typing and any new consumer.)_

**Recommendation:** A — express the real tuple shapes in the type so consumers handle reverse primitives correctly.

**Decision:** A — widen the `Primitive` type alias to the real tuple shapes.

### Offline — validator

#### 46. NaN passes every numeric check (coords, obstacle bounds, headings, scalars)

- **Location:** `tools/path_planner/validator.py:411`
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** field data validation
- **Traced path:** `validate() -> _is_number(float('nan')) returns True -> obstacle bounds check x_min >= x_max is False for NaN -> NaN bounds accepted`

**What's wrong:** `_is_number()` (53-55) returns True for `float('nan')` (it is a float and not a bool). Every
numeric gate then lets NaN through: NODES coords (324-329), OBSTACLES bounds (`x_min >= x_max` is always False
when an operand is NaN, so a NaN rectangle passes, line 411), `ROBOT_RADIUS_CM` (`<= 0` False, 418),
`COST_BLIND_MULTIPLIER` (422), `FIELD_MM` (`<= 0` False, 448), and EDGES distance/heading. (Only
`START_HEADING_DEG` is rejected, because `0.0 <= nan < 360.0` is False.) So NaN slips past the geometry checks
and reaches `inflate_obstacles()`/Euclidean math, producing NaN edge weights and a corrupt planner graph with no
diagnostic.

**Options:**
- **A —** Make `_is_number` reject non-finite values (`math.isfinite`), so NaN/Inf fail every numeric gate with a clean FieldDataError. _(One-line fix at the root; covers all gates at once.)_
- **B —** Add explicit `math.isnan` checks at each numeric gate. _(More verbose; easy to miss a gate — exactly how this slipped.)_
- **C —** Leave as-is. _(NaN surfaces as corrupt edge weights deep in Dijkstra instead of a precise validation error.)_

**Recommendation:** A — require finiteness in `_is_number`; it fixes every gate in one place.

**Decision:** A — require finite numbers in `_is_number` (fixes every numeric gate at once).

#### 47. EDGES entry duplicating a LINE/BLIND pair with a conflicting type is not detected (validator side)

- **Location:** `tools/path_planner/validator.py:461`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** field data validation
- **Traced path:** `validate() -> cross-check (378-384) covers only LINE_SEGMENTS vs BLIND_EDGES -> _validate_edges_field() validates EDGES in isolation -> no cross-check against LINE_SEGMENTS/BLIND_EDGES`

**What's wrong:** The validator enforces that a pair cannot appear in both LINE_SEGMENTS and BLIND_EDGES
(376-384), but `_validate_edges_field` never cross-checks an EDGES entry against them. An EDGES entry like
`{type: EDGE_BLIND, u: A, v: B}` where A-B is already a LINE_SEGMENTS pair passes validation, and in
`graph_builder` the later `add_edge` overwrites the earlier edge's type/attrs (the same corruption as #38, seen
from the validation layer). The anti-ambiguity rule the validator enforces for LINE-vs-BLIND is bypassable
through EDGES.

**Options:**
- **A —** Extend the validator's duplicate-pair cross-check to also reject an EDGES pair that conflicts with an existing LINE/BLIND pair. _(Fixes #38 and #47 together at the validation layer — the clean place.)_
- **B —** Allow the EDGES override only when its type matches the existing pair (override-not-redefine). _(Encodes intent; needs override semantics.)_
- **C —** Leave as-is. _(Silent reclassification of a segment via EDGES; hard to spot.)_

**Recommendation:** A — one cross-check in the validator resolves both this and #38.

**Decision:** A — validator cross-check (same fix as #38).

### Offline — header_writer

#### 48. C++ enum bodies in routes.h are hardcoded strings, decoupled from the Python enums they mirror

- **Location:** `tools/path_planner/header_writer.py:331`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** pipeline configuration
- **Traced path:** `_build_routes_h_lines() emits enum bodies as fixed string literals (331-353) -> _emit_pipeline_config() emits PIPELINE_CONFIG.<field>.name (468-472) with no cross-check that the name exists in the hardcoded routes.h enum`

**What's wrong:** The C++ enum definitions written into routes.h (LineMode/BlindMode/ProjectionMode/TurnMode/
DirectionMode) are hand-written string literals in `_build_routes_h_lines`, independent of the `IntEnum`
definitions in `pipeline_config.py`. `_emit_pipeline_config` bakes whatever `PIPELINE_CONFIG.<field>.name` is into
routes.cpp verbatim. If a Python enum gains or renames a member that gets selected in PIPELINE_CONFIG, routes.cpp
references an enum literal (e.g. `LineMode::TURBO`) that the hardcoded routes.h enum does not declare. There is no
Python-side guard and no test asserting the two member-name sets match — the failure surfaces only at
arduino-compile, not at build time.

**Options:**
- **A —** Generate the C++ enum bodies *from* the Python enums (single source of truth) instead of hardcoding them. _(Eliminates the drift entirely; the proper fix.)_
- **B —** Add a build-time assertion that every member name used by PIPELINE_CONFIG exists in the hardcoded routes.h enum set. _(Cheaper; catches the drift at `build_routes.py` time with a clear message.)_
- **C —** Leave as-is. _(A new/renamed mode silently produces a routes.cpp that fails to compile downstream.)_

**Recommendation:** A — emit the enums from the Python definitions so routes.h and PIPELINE_CONFIG can never diverge.

**Decision:** A — generate the C++ enum bodies from the Python enums (B build-time assert acceptable as a cheaper interim).

#### 49. Node names are emitted into C++ string literals without escaping quotes/backslashes

- **Location:** `tools/path_planner/header_writer.py:252`
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** C++ header writer (routes.h / routes.cpp emitter)
- **Traced path:** `emit_routes() -> _emit_nodes_array() -> _node_row() where quoted = '"' + name + '"'`

**What's wrong:** `_node_row` builds the `NodeInfo.name` field as `'"' + name + '"'` with no escaping. A node name
containing a double-quote, backslash, or newline produces a malformed C++ literal (name `A"B` emits `"A"B"`,
which won't compile; a backslash emits an invalid escape). The validator only checks that a NODES key is a
non-empty string (320-323) — it never rejects quotes/backslashes/non-printables — so nothing upstream guards
this. `build_routes.py` emits routes.cpp successfully, but the Arduino build then fails on the generated
(do-not-hand-edit) file rather than pointing at the field data.

**Options:**
- **A —** Escape the name when emitting (or restrict NODES names to a safe charset in the validator and reject the rest with a FieldDataError). _(Validator-side rejection is the cleaner UX — error points at the field data, not the generated file.)_
- **B —** Escape in `_node_row` only (`name.replace('\\','\\\\').replace('"','\\"')`). _(Makes any name compile, but allows odd names downstream.)_
- **C —** Leave as-is. _(A stray quote in a node name yields a confusing compile error in an auto-generated file.)_

**Recommendation:** A — reject unsafe names in the validator so the error surfaces at the right layer.

**Decision:** A — reject unsafe node names (quotes/backslashes/non-printables) in the validator.

#### 50. No guard against ≥255 nodes: index values collide with `NEXT_HOP_NONE` (0xFF) and overflow `uint8_t`

- **Location:** `tools/path_planner/header_writer.py:309`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** C++ header writer (routes.h / routes.cpp emitter)
- **Traced path:** `_build_routes_h_lines() emits constexpr uint8_t N_NODES and NEXT_HOP_NONE = 0xFF; _emit_next_hop_table() emits str(index) into a uint8_t[][] array`

**What's wrong:** `N_NODES` is `constexpr uint8_t` and NEXT_HOP cells / START_NODE are emitted as plain decimal
indices into `uint8_t` fields, with `NEXT_HOP_NONE` fixed at 0xFF (255). With ≥256 nodes `N_NODES` overflows
`uint8_t`; with exactly 256, node index 255 is numerically identical to the `NEXT_HOP_NONE` sentinel, so a
legitimate next-hop to node 255 is indistinguishable from "unreachable." The emitter writes the true decimal
into the `uint8_t` array with no warning, silently truncated by the C++ compiler. No assertion anywhere checks
`n_nodes < 255`.

**Options:**
- **A —** Assert `n_nodes < 255` in `emit_routes` (fail the build with a clear message). _(Cheap ceiling guard; current field is <50 nodes so it never fires today.)_
- **B —** Widen the index type to `uint16_t` and move the sentinel out of valid range. _(Removes the ceiling entirely; touches routes.h types and firmware indexing.)_
- **C —** Leave as-is. _(Unreachable for the WRO field; a latent hard ceiling with no diagnostic.)_

**Recommendation:** A — a one-line assert documents and enforces the ceiling at build time.

**Decision:** A — assert `n_nodes < 255` at build time (one line).

### Offline — pipeline_config

#### 51. PIPELINE_CONFIG fields are never validated; a self-referential DEFAULT silently degrades to a hardcoded fallback

- **Location:** `tools/path_planner/pipeline_config.py:62`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** pipeline configuration
- **Traced path:** `PipelineConfig (no __post_init__) -> build_routes._run() validates only field_data -> _emit_pipeline_config() bakes LineMode::DEFAULT -> C++ _resolveLineMode() returns DEFAULT -> _lineModeToPreset(DEFAULT) falls through to NORMAL`

**What's wrong:** `PipelineConfig` has no validation, and `build_routes._run()` validates only `field_data`, never
the pipeline config singleton. Setting any of `line`/`blind`/`projection`/`turn` to its own `.DEFAULT` member is
circular (DEFAULT means "inherit `PIPELINE_CONFIG.<category>`," i.e. inherit from itself). This passes the whole
pipeline, emits e.g. `LineMode::DEFAULT` into the C++ struct, and at dispatch the `_resolve*Mode` helpers return
DEFAULT, which the `*ModeToPreset`/`_turnModeToByte` switch silently maps to NORMAL/COUNTER_ROT. The
`field_data.EDGES` validator explicitly rejects DEFAULT overrides for exactly this confusion, but no equivalent
guard exists for PIPELINE_CONFIG.

**Options:**
- **A —** Add a `__post_init__` (or a check in `build_routes._run()`) rejecting `*.DEFAULT` on the PIPELINE_CONFIG category fields, mirroring the EDGES validator. _(Closes the circular config with a clear error.)_
- **B —** Make DEFAULT resolve to the documented concrete default (NORMAL/COUNTER_ROT) explicitly and warn. _(Accepts DEFAULT but makes the result intentional; weaker.)_
- **C —** Leave as-is. _(A circular config silently runs the hardcoded fallback with no signal.)_

**Recommendation:** A — reject self-referential DEFAULT on PIPELINE_CONFIG, matching the EDGES rule.

**Decision:** A — reject self-referential DEFAULT on PIPELINE_CONFIG (mirror the EDGES rule).

### Offline — visualizer

#### 52. `imshow(aspect="auto")` clobbers `set_aspect("equal")`, distorting the rendered map

- **Location:** `tools/path_planner/visualizer.py:160`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** field map visualizer
- **Traced path:** `render_map() sets ax.set_aspect("equal") (105) -> _draw_background() calls ax.imshow(..., aspect="auto") (154-161) -> fig.savefig()`

**What's wrong:** `render_map()` establishes a 1:1 data aspect with `ax.set_aspect("equal")` (105) so coordinates
(cm) map proportionally. `_draw_background()` then draws the field PNG with `ax.imshow(..., aspect="auto")`. In
matplotlib, `imshow(aspect="auto")` internally calls `ax.set_aspect("auto")`, silently overriding the earlier
`"equal"`. With aspect released, data units scale independently on x and y to fill the axes box; after
`savefig(..., bbox_inches="tight")` the box ratio is set by margins/title/legend, not `width_cm:height_cm`, so
edges, inflated-obstacle polygons, node positions, and C-space rings are stretched and no longer proportional —
exactly the project's recorded matplotlib gotcha. The tests only assert PNG file size (>1024 bytes), so they
never catch the distortion. Fix: omit the `aspect` kwarg on `imshow`, or pass `aspect="equal"`.

**Options:**
- **A —** Drop the `aspect="auto"` kwarg on the `imshow` call (or pass `aspect="equal"`), so the earlier `set_aspect("equal")` survives. _(The exact recorded fix; one kwarg.)_
- **B —** Re-assert `ax.set_aspect("equal")` after the `imshow`. _(Also works; relies on call ordering, more fragile.)_
- **C —** Leave as-is. _(Every map render on a non-square field is geometrically distorted; not advisable for a measurement aid.)_

**Recommendation:** A — remove the `aspect` kwarg on `imshow`; it is the documented matplotlib gotcha fix.

**Decision:** A — drop the `aspect` kwarg so the reviewed `map.png` stays true-aspect (P3.6).

#### 53. `scatter()` emits per-type legend labels that `_attach_legend()` silently discards

- **Location:** `tools/path_planner/visualizer.py:255`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** field map visualizer
- **Traced path:** `_draw_nodes() ax.scatter(..., label=f"node: {node_type}") -> _attach_legend() ax.legend(handles=...)`

**What's wrong:** `_draw_nodes()` attaches a label to every `ax.scatter()` (257), implying those feed the legend.
But `_attach_legend()` calls `ax.legend(handles=edge_handles + obstacle_handles + node_handles)` — an explicit
handles list — which makes matplotlib ignore auto-collected scatter labels. The node legend entries come from the
hand-built `node_handles` derived from `_NODE_COLOR`, not the scatter labels. A node with a type absent from
`_NODE_COLOR` is drawn gray and labeled by scatter, but would still be missing from the legend. A latent
legend-completeness gap, not a crash.

**Options:**
- **A —** Either drop the misleading `label=` from `scatter` (legend is hand-built), or build the legend from the actual node types present rather than only `_NODE_COLOR`. _(Pick one source of truth for legend entries.)_
- **B —** Add an assertion/warning when a node type is missing from `_NODE_COLOR`. _(Surfaces the gap; doesn't fix the dead label.)_
- **C —** Leave as-is. _(Cosmetic today — validator restricts node types to line/blind, both in `_NODE_COLOR`.)_

**Recommendation:** A — remove the dead `label=` so the code says what it does; revisit if a new node type is added.

**Decision:** A — remove the dead `label=` from `scatter`.

### Offline — sim/runtime

#### 54. `go_to`/`step_through` walk NEXT_HOP with no step cap (unbounded loop on a cyclic table)

- **Location:** `tools/path_planner/sim/runtime.py:678`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** PathTracker Python simulator
- **Traced path:** `PathTrackerSim.go_to() -> while current != goal_name: nxt = self._next_hop[current][goal_name] (no iteration bound)`

**What's wrong:** Both `go_to()` (678) and `step_through()` (808) drive a bare `while current != goal_name` loop
that re-indexes `self._next_hop[current][goal_name]` with no max-step guard. The sibling `walk_next_hop()` (234)
*does* cap at `len(next_hop) + 1` and raises ValueError. If the next_hop table contains a cycle that never
reaches the goal (a hand-rolled/corrupted table where `next_hop[X][goal]=Y` and `next_hop[Y][goal]=X` but
neither is the goal), `go_to`/`step_through` spin forever. The None branch catches unreachable, not
cyclic-but-non-None.

**Options:**
- **A —** Add the same `len(next_hop) + 1` iteration cap (raise on exceed) that `walk_next_hop` already uses. _(Consistent with the sibling helper; turns a hang into a clear error.)_
- **B —** Track visited nodes and raise on a revisit. _(Detects cycles precisely; slightly more state.)_
- **C —** Leave as-is. _(A corrupted/hand-built table hangs the sim instead of faulting.)_

**Recommendation:** A — reuse the existing cap pattern for parity with `walk_next_hop`.

**Decision:** Defer — sim-only; add the `len(next_hop)+1` step cap when sim hardening is worthwhile.

#### 55. Pass-through still emits a residual `ACT_TURN_REL` when collinearity is within tolerance but not exactly 0

- **Location:** `tools/path_planner/sim/runtime.py:724`
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** PathTracker Python simulator
- **Traced path:** `go_to() -> _should_pass_through() True for |delta|<=NAV_TURN_TOLERANCE_DEG -> emit ACT_LINE_PASS -> next iteration recomputes turn delta = wrap_pm180(...) still > TURN_EPSILON_DEG`

**What's wrong:** `_should_pass_through()` (539) fires pass-through whenever the body-heading delta is `<=
NAV_TURN_TOLERANCE_DEG` (0.5°), i.e. not exactly zero. After `ACT_LINE_PASS`, the next loop iteration computes
its own turn delta against `TURN_EPSILON_DEG` (1e-6, line 700). For any real collinear-within-tolerance case
(e.g. segments 0.3° apart), that delta is 0.3° > 1e-6, so an `ACT_TURN_REL(0.3)` is emitted right after the
pass-through — contradicting the "drive straight through" intent. The TC9 test only uses exactly-0° fixtures, so
this window is uncovered.

**Options:**
- **A —** After emitting a pass-through, advance the sim's heading to the new edge heading (or suppress the next turn within `NAV_TURN_TOLERANCE_DEG`) so no residual micro-turn is emitted. _(Matches firmware intent; needs the sim to model the pass-through heading update.)_
- **B —** Raise `TURN_EPSILON_DEG` to `NAV_TURN_TOLERANCE_DEG` for the post-pass-through tick. _(Simple but conflates two different thresholds.)_
- **C —** Leave as-is and add a TC covering near-collinear fixtures to document the behavior. _(At least surfaces it; doesn't fix the stray turn.)_

**Recommendation:** A — model the heading update on pass-through so the sim mirrors the robot's "no turn" behavior.

**Decision:** Defer — sim-only; revisit at Phase 5 sim-parity.

### Offline — pick_xy

#### 56. `'c'` clear-markers key collides with matplotlib's default `keymap.back`

- **Location:** `tools/path_planner/pick_xy.py:159`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** coordinate picker GUI  *(merged: found under two capability labels)*
- **Traced path:** `main() -> mpl_connect('key_press_event', _on_key) -> _on_key(event.key=='c') runs alongside matplotlib's built-in keymap.back handler`

**What's wrong:** The custom handler treats `'c'` as "clear every picker marker." But matplotlib's default
`rcParams['keymap.back']` is `['left', 'c', 'backspace', 'MouseButton.BACK']`. Registering a custom
`key_press_event` callback does not disable the built-in keymap, so a single `'c'` press fires *both*: the picker
clears its markers AND matplotlib executes the toolbar "back" navigation (pop the view stack). After panning or
zooming, pressing `'c'` to clear also snaps the view back to a prior zoom/pan state. (`'u'` undo is safe — not in
any default keymap.)

**Options:**
- **A —** Remove `'c'` from `rcParams['keymap.back']` (or clear the relevant keymap entries) at startup, or rebind the clear action to a key not in any default keymap. _(Stops the double-fire; pick whichever is least surprising to operators.)_
- **B —** Check `event.key` against the toolbar state / consume the event so the built-in handler doesn't also run. _(Harder to do cleanly with mpl's event model.)_
- **C —** Leave as-is and document that `'c'` also navigates back. _(Confusing for a measurement tool; not advisable.)_

**Recommendation:** A — strip `'c'` from `keymap.back` at startup (or move clear to a free key).

**Decision:** A — free the `'c'` clear key from matplotlib's `keymap.back`.

#### 57. `imshow(aspect="auto")` stretches the field photo in the interactive figure (no compensating figsize)

- **Location:** `tools/path_planner/pick_xy.py:210`
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** coordinate picker GUI
- **Traced path:** `main(): ax.set_aspect("equal") [197] then ax.imshow(..., aspect="auto") [210] releasing the equal-aspect lock`

**What's wrong:** Same matplotlib gotcha as #52: `ax.imshow(..., aspect="auto")` calls `ax.set_aspect("auto")`,
overriding the `set_aspect("equal")` at line 197. Unlike `visualizer.py`, which compensates with
`figsize=(10, 10*height_cm/width_cm)`, `main()` here calls `plt.subplots()` with no figsize, so the default ~4:3
figure stretches the 320×154 cm field (FIELD_MM 3200×1540) non-uniformly, and every window resize re-stretches it.
The printed coordinates stay correct (read from the data transform; xlim/ylim unchanged), but the background photo
and grid are visually distorted — so an operator measuring by eye aligns clicks against a stretched image.

**Options:**
- **A —** Drop `aspect="auto"` on the `imshow` (or pass `aspect="equal"`) so the field stays proportional. _(Same fix as #52; primary.)_
- **B —** Set `figsize` to the field aspect like `visualizer.py` does. _(Helps the initial view but resizes still stretch unless aspect is locked; do A as well.)_
- **C —** Leave as-is. _(Defeats the tool's purpose of visually-accurate measurement.)_

**Recommendation:** A — remove the `aspect` kwarg so the equal-aspect lock holds and the photo isn't stretched.

**Decision:** A — drop the `aspect` kwarg so pick_xy shows a true-aspect photo (accurate P3.4 measuring).

#### 58. Missing/corrupt `field_data.json` crashes at import, bypassing `main()`'s graceful exit-code 2 handling

- **Location:** `tools/path_planner/pick_xy.py:46`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** coordinate picker GUI
- **Traced path:** `import field_data [46] -> field_data.py:34 load_field_json() -> raises FieldDataError before main() runs`

**What's wrong:** `pick_xy.py` imports `field_data` at module top (46). `field_data.py` runs `load_field_json()` at
import time (line 34), which raises `FieldDataError` when the JSON is missing or invalid. That propagates out of
the import before `main()` runs, so the script dies with a raw traceback. `main()` promises "Returns 0 on clean
exit, 2 if map.png is missing" and guards an unreadable map, but there is no equivalent guard for an
absent/corrupt `field_data.json`.

**Options:**
- **A —** Wrap the `field_data` import in a try/except (or defer the load into `main()`), printing a clean error and returning 2. _(Brings field_data failures into the documented graceful path.)_
- **B —** Make `field_data.py` not load at import time (lazy load). _(Broader change; affects all importers.)_
- **C —** Leave as-is. _(Operator running before the editor generates the JSON gets a stack trace instead of a clean exit.)_

**Recommendation:** A — guard the import so the failure mode matches the documented exit-code-2 contract.

**Decision:** A — guard the `field_data` import; clean exit 2 instead of a traceback.

#### 59. Pan/zoom toolbar drags spuriously print coordinates and drop markers

- **Location:** `tools/path_planner/pick_xy.py:118`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** interactive field coordinate picker (pick_xy)
- **Traced path:** `_on_click(event) where event.button==1 and event.inaxes is ax -> prints coord + ax.scatter() + ax.annotate() regardless of toolbar mode`

**What's wrong:** The click handler filters only on `event.button==1` and `event.inaxes`; it never checks
`fig.canvas.toolbar.mode`. When the navigation toolbar is in pan or zoom mode, the initial mouse-down of a drag
still emits a `button_press_event` with `button==1` inside the axes, so the handler prints an `(x,y)` line and
adds a red marker even though the operator meant to pan/zoom — polluting the coordinates being copy-pasted into
NODES and leaving a stray dot.

**Options:**
- **A —** Guard the click handler with `if fig.canvas.toolbar and fig.canvas.toolbar.mode: return` so pan/zoom drags don't register as measurements. _(Standard mpl pattern; small.)_
- **B —** Only register a click on `button_release_event` with no intervening drag. _(More robust against drags; larger change.)_
- **C —** Leave as-is. _(Stray markers/coords whenever the operator zooms; annoying.)_

**Recommendation:** A — check the toolbar mode before treating a click as a measurement.

**Decision:** A — ignore clicks while the pan/zoom toolbar is active (no stray markers/coords).

#### 60. Corrupt/undecodable `map.png` escapes the narrow `imread` except clause; malformed NODES coord crashes startup

- **Location:** `tools/path_planner/pick_xy.py:187` (and `:93` for the NODES-overlay shape gap)
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** interactive field coordinate picker (pick_xy)
- **Traced path:** `imread raises ValueError/SyntaxError -> except (FileNotFoundError, PermissionError, OSError) misses it ; _draw_nodes_overlay does coord[0]/coord[1] + (x,y) unpack with no shape guard`

**What's wrong:** Two robustness gaps in pick_xy startup, grouped here. (1) `imread` is wrapped in
`except (FileNotFoundError, PermissionError, OSError)` (line 187). A file that exists but isn't a decodable image
makes matplotlib/Pillow raise `ValueError`/`SyntaxError` (not OSError), which escapes and crashes `main()` instead
of the intended clean "cannot read map" exit 2; the sibling `visualizer.py` uses a broad `except Exception` here.
(2) `_draw_nodes_overlay` (line 93) does `coord[0]`/`coord[1]` and unpacks `(x, y)` with no shape guard; the loader
only tuple-izes length-2 lists, and pick_xy reads `field_data` without running `validator.validate`, so a node
with a length-1/3 coord, a scalar, or null raises `IndexError`/`TypeError` and aborts the GUI before `plt.show()`.

**Options:**
- **A —** Broaden the `imread` except to `except Exception` (match `visualizer.py`) and add a shape guard in `_draw_nodes_overlay` (skip + warn on any non-length-2 coord) so a mid-edit `field_data.json` opens for repair. _(Covers both gaps; both are cheap.)_
- **B —** Run `validator.validate` (or a lighter shape check) on `field_data` before drawing, surfacing a clean error. _(Stricter; refuses to open on bad data instead of skipping the bad node.)_
- **C —** Leave as-is. _(A truncated PNG or a half-edited node coord dumps a stack trace instead of a usable picker.)_

**Recommendation:** A — broaden the except and guard the overlay so the picker degrades gracefully during editing.

**Decision:** A — broaden the `imread` except + guard the overlay coord shape.

### Offline — editor

#### 61. Drag released outside the canvas axes leaves an unrecorded, un-undoable node move

- **Location:** `tools/path_planner/editor/app.py:718` (release path `:725`)
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** live field editor GUI / interactive field editor (edit_field)  *(merged: found under two labels, lines 718 & 725)*
- **Traced path:** `_handle_move_click() latches _drag_target -> _on_canvas_drag() live-mutates model.nodes[target] (718) -> mouse released OUTSIDE axes -> FieldCanvas._on_button_release() sees figure_to_world()==None and returns early (canvas.py:442-446), never calling on_release -> _finish_move() never runs (725)`

**What's wrong:** In MOVE mode, `_on_canvas_drag` writes the live cursor position directly into
`model.nodes[self._drag_target]` on every motion event (718), bypassing the History/Command stack. The permanent
`MoveNode` command and the restore-then-record bookkeeping only happen in `_finish_move`, reached via
`_on_canvas_release → on_release`. But `FieldCanvas._on_button_release` (canvas.py:442-446) returns early
(`coords is None`) when the release lands outside the axes — exactly a release over the side panel/window chrome —
so `on_release` is never called. It still clears `_active_button`. Result: the dragged coordinate is committed to
the model with no `MoveNode` entry, no dirty/undo bookkeeping, and `_drag_target`/`_drag_origin_xy` left dangling.
The node has silently moved and cannot be undone. Releasing off the plot is a very common gesture when dragging
toward the field edge.

**Options:**
- **A —** In `FieldCanvas._on_button_release`, still call `on_release` on an off-axes release (clamp coords to the axes or pass the last in-axes position) so `_finish_move` always records a `MoveNode`. _(Fixes the missing command at the canvas boundary; the editor's release logic then runs normally.)_
- **B —** In `_on_canvas_drag`, don't mutate `model.nodes` live; instead move only a visual proxy and apply the `MoveNode` command on a valid release. _(Removes the live in-place mutation entirely — cleaner model invariants — but a larger refactor of the drag visuals.)_
- **C —** Leave as-is. _(A common sloppy drag silently moves a node with no undo and possibly no dirty flag; data-integrity risk.)_

**Recommendation:** A — ensure `on_release`/`_finish_move` runs even for an off-axes release so every drag records a command.

**Decision:** A — run `_finish_move` even on an off-axes release so every drag records a `MoveNode`.

#### 62. Deleting the start node leaves `model.start_node` dangling and blocks every later Save

- **Location:** `tools/path_planner/editor/commands.py:359`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** live field editor / interactive field editor (edit_field)  *(merged: found under two labels)*
- **Traced path:** `_context_delete(start) -> DeleteNode.apply() removes node but never checks/updates model.start_node -> model.start_node dangles -> later _save() -> validate() raises "START_NODE references unknown node"`

**What's wrong:** `DeleteNode.apply` (359-390) only refuses when `line_segments`/`blind_edges` reference the node;
it does not check or update `model.start_node` (whereas `RenameNode` cascades into `start_node`). After deleting
the node that `start_node` points to, `model.start_node` references a missing name; `panels.refresh` then sets the
readonly start-node combobox to a value not in its list (panels.py:518). The inconsistency is only caught at the
next `_save()`, where `validate()` raises `START_NODE references unknown node` and aborts the save — the operator
is stuck (can't save until they re-pick a start node), and the error points at START_NODE rather than the delete.

**Options:**
- **A —** In `DeleteNode.apply` / `_context_delete`, refuse to delete the current start node (or require choosing a new start first) with a clear message. _(Prevents the broken state up front; clearest UX.)_
- **B —** On deleting the start node, clear/auto-reassign `model.start_node` (e.g. to another node or empty) and mark it for re-selection. _(Allows the delete but forces a follow-up; risk of a silently wrong start.)_
- **C —** Leave as-is. _(Operator can reach an unsaveable state with a confusing error; not advisable.)_

**Recommendation:** A — guard the delete of the start node, matching how the model treats start_node as load-bearing.

**Decision:** A — guard delete of the current start node.

#### 63. Renaming a node makes the EDGES delete-guard miss references, allowing deletion of an EDGES-referenced node

- **Location:** `tools/path_planner/editor/app.py:1189`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** live field editor GUI
- **Traced path:** `_context_rename(X->Y) does NOT update cached field_data.EDGES -> _context_delete(Y) -> _find_edges_reference(Y) scans EDGES still holding 'X' -> returns None -> guard passes -> DeleteNode(Y) succeeds`

**What's wrong:** `_find_edges_reference` (1189-1209) scans the cached `field_data` module's live EDGES list, only
refreshed after a successful Save (importlib.reload at 1329). After an in-session rename X→Y (pending, not
flushed), EDGES still references 'X'. Deleting the renamed 'Y' makes the guard search EDGES for 'Y', find nothing,
and allow the delete. EDGES is then left referencing a node ('X') that no longer exists in the JSON model, which
the next build rejects. The delete-guard's purpose (block deletes of EDGES-referenced nodes) is defeated for any
node renamed earlier in the session.

**Options:**
- **A —** Have `_find_edges_reference` also consult the pending-rename map (resolve Y back to X before scanning), or apply pending renames to the cached EDGES before the guard runs. _(Makes the guard see the current name; the targeted fix.)_
- **B —** Refresh the cached EDGES on each rename rather than only on Save. _(Keeps the cache current; more reloads.)_
- **C —** Leave as-is. _(Renamed-then-deleted nodes slip past the guard and break the next build.)_

**Recommendation:** A — resolve through the pending-rename map in the guard so it checks the right name.

**Decision:** A — resolve through the pending-rename map in the delete guard.

#### 64. Pending renames re-accumulate from history after a Save, re-marking dirty and re-triggering EDGES rewrites

- **Location:** `tools/path_planner/editor/app.py:1339`
- **Severity:** suspicious   **Scope:** pre-existing
- **Capability:** live field editor GUI
- **Traced path:** `_save() clears _pending_renames but NOT _history -> later _undo()/_redo() -> _recompute_pending_renames() walks history.undo_stack, replays surviving RenameNode commands -> _pending_renames repopulated with already-saved renames -> next _save() re-attempts EDGES rewrite`

**What's wrong:** `_save` clears `_pending_renames` (1339) but never clears the History; `RenameNode` commands stay
on the undo stack. Any later undo/redo calls `_recompute_pending_renames` (1250-1265), which rebuilds
`_pending_renames` from the entire surviving undo stack — re-adding renames already flushed in the prior Save. The
next Save re-enters the `ast_rewrite` path for them. Usually `rewrite_edges_uv` is a no-op (the old name no longer
exists, so `transformer.changed` stays False), but the editor still treats the buffer as needing a rename flush
and the unconditional `_mark_dirty` in undo/redo keeps the window dirty. If a node with a previously-renamed old
name is later re-created, the stale re-accumulated mapping could rewrite an unrelated EDGES entry (overlaps #65).

**Options:**
- **A —** After a Save, drop the flushed `RenameNode` commands from history (or snapshot a "last saved" marker so `_recompute_pending_renames` only collects renames *after* the last Save). _(Prevents re-accumulation; the principled fix.)_
- **B —** Make `_recompute_pending_renames` skip renames whose old name no longer exists in the model. _(Stops most re-rewrites; doesn't fix the spurious dirty flag.)_
- **C —** Leave as-is. _(Mostly benign no-op rewrites + a sticky dirty flag, with a latent corruption tie-in to #65.)_

**Recommendation:** A — anchor pending-rename recomputation to the last Save so flushed renames don't return.

**Decision:** A — anchor pending-rename recompute to the last Save.

#### 65. Reusing a former node name for a new node corrupts the pending-rename collapse map

- **Location:** `tools/path_planner/editor/app.py:1243`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** live field editor GUI
- **Traced path:** `_record_pending_rename('A','B') -> {A:B}; user adds a fresh node also named 'A'; _record_pending_rename('A','C') -> fresh branch overwrites pending['A']=C, discarding {A:B}`

**What's wrong:** `_record_pending_rename` (1211-1248) keys the pending map on the original (old) name. If the
operator renames A→B (pending `{A:B}`) and later creates a brand-new node also named 'A', then renames that new
'A'→'C', the forwarding loop only looks for a pending entry whose *target* equals 'A' (none — the target is 'B'),
so it falls into the fresh branch and executes `pending['A']='C'`, clobbering the original `{A:B}`. The Save-time
EDGES rewriter then renames the original 'A' references to 'C' instead of 'B', corrupting the edge table. The
collapse logic assumes node names are never reused after a rename within a session.

**Options:**
- **A —** Key the pending-rename map on a stable node identity (not the mutable name), so reusing a freed name can't collide. _(Robust fix; requires a node id distinct from its display name.)_
- **B —** Forbid reusing a name that is the source of a pending rename until Save, with a clear message. _(Prevents the collision; mildly restrictive.)_
- **C —** Leave as-is. _(A plausible reorg workflow silently corrupts the EDGES table.)_

**Recommendation:** B as an immediate guard; A if the model can carry a stable node id.

**Decision:** B — forbid reusing a pending-rename source name until Save.

#### 66. Zero field width crashes the editor at startup with `ZeroDivisionError`

- **Location:** `tools/path_planner/editor/app.py:272`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** live field editor GUI
- **Traced path:** `EditorApp.__init__() -> load_field_json() (no validation) -> FieldModel.from_json_dict() (no validation) -> _initial_window_geometry() -> _field_aspect_ratio() returns 0/H=0.0 -> canvas_w / aspect divides by 0`

**What's wrong:** `_field_aspect_ratio` (258-270) guards only `height_mm <= 0` (returns 1.0), not `width_mm`. With
`FIELD_MM = [0, H]`, aspect = 0/H = 0.0, and `_initial_window_geometry` (282) then computes `canvas_w / aspect`,
raising `ZeroDivisionError`. The constructor loads via `load_field_json` + `from_json_dict`, neither of which runs
`validator.validate` (validation only happens on Save), so a degenerate `FIELD_MM` reaches the sizing math and
crashes before the window is built.

**Options:**
- **A —** Guard `width_mm <= 0` in `_field_aspect_ratio` (return a safe default like 1.0) just as `height_mm` is guarded. _(One-line symmetry fix; the editor opens so the bad value can be corrected.)_
- **B —** Run a lightweight schema check on load (not full validate) that rejects non-positive FIELD_MM with a dialog. _(Clearer error; more code in the constructor.)_
- **C —** Leave as-is. _(A hand-edited zero width crashes the editor on launch instead of surfacing the error.)_

**Recommendation:** A — mirror the height guard for width so the editor degrades gracefully.

**Decision:** A — guard `width_mm <= 0` in `_field_aspect_ratio`.

#### 67. Keyboard shortcuts are bound globally (`bind_all`) and fire while typing in dialogs/entry fields

- **Location:** `tools/path_planner/editor/app.py:358`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** interactive field editor GUI (edit_field)
- **Traced path:** `_wire_keyboard_shortcuts() uses root.bind_all('<KeyPress-l>'/'<KeyPress-b>'/'<Delete>'/Ctrl chords) -> typing 'l'/'b' in the rename simpledialog or an entry triggers _handle_edge_*/_handle_delete_key`

**What's wrong:** `_wire_keyboard_shortcuts` binds `l`/`L`/`b`/`B`/`Delete` and the Ctrl chords with `bind_all` on
the root, so they fire for any widget including text entries and modal dialogs. The L/B handlers check
`_current_mode() == Mode.EDGE` and a 2-node selection, so in non-EDGE modes they're inert. But in EDGE mode, if
the operator opens the rename dialog and types a literal 'l' or 'b', the global binding still fires
`_handle_edge_line_key`/`_handle_edge_blind_key`, dispatching `AddLineEdge`/`AddBlindEdge` on the selected pair
behind the modal. Ctrl-S/Ctrl-Z likewise fire while a dialog is open. The handlers don't check whether keyboard
focus is on the canvas.

**Options:**
- **A —** Gate the global handlers on keyboard focus being the canvas (return early if focus is an Entry/dialog), or bind to the canvas widget rather than `bind_all`. _(Stops surprise edits while typing; standard Tk fix.)_
- **B —** Disable the shortcuts while a modal dialog is open (push/pop bindings around the dialog). _(Covers the dialog case; misses plain entry fields.)_
- **C —** Leave as-is. _(Typing a node name with 'l'/'b' in EDGE mode silently adds an edge behind the dialog.)_

**Recommendation:** A — scope the shortcuts to canvas focus so they don't fire during text entry.

**Decision:** A — scope keyboard shortcuts to canvas focus.

### Offline — migrate / loader

#### 68. EDGES-tail fallback matches the substring inside `BLIND_EDGES = [`, swallowing real attribute literals into the verbatim tail

- **Location:** `tools/path_planner/editor/migrate.py:312`
- **Severity:** bug   **Scope:** pre-existing
- **Capability:** field data JSON migration
- **Traced path:** `run() -> _extract_edges_tail(source) -> (banner missing) -> source.find("EDGES = [") matches inside "BLIND_EDGES = ["`

**What's wrong:** The fallback locates the EDGES table via `source.find("EDGES = [")`. That token is also a literal
substring of `BLIND_EDGES = [`, which appears earlier. So `find` returns the index of `EDGES = [` *inside*
`BLIND_EDGES = [`, and `_extract_edges_tail` walks back to that line start and returns everything from
`BLIND_EDGES = [` downward as the "verbatim tail." `_build_rewritten_source` concatenates the generated header
(which defines `BLIND_EDGES`, `OBSTACLES`, `ROBOT_RADIUS_CM`, `COST_BLIND_MULTIPLIER`, `START_NODE`,
`START_HEADING_DEG` as JSON-backed aliases) with this tail, which *re-defines* those same names as the OLD Python
literals. The literal re-definitions execute after the aliases and shadow them, so the migrated module silently
keeps the pre-migration values instead of reading from `field_data.json`, with duplicate definitions in the file.
The banner path is the primary anchor; any author edit to the banner (dash count, blank line, reworded "Rich edge
list") drops into this unanchored fallback. (The idempotency probe `_NODES_LITERAL_RE` is correctly line-anchored;
this token search is not — the root cause.)

**Options:**
- **A —** Anchor the fallback token search to a line start (regex `^\s*EDGES\s*=\s*\[` with `re.MULTILINE`), so it can't match inside `BLIND_EDGES`. _(Directly fixes the root cause; mirrors the anchored `_NODES_LITERAL_RE`.)_
- **B —** Search for `\nEDGES = [` (newline-prefixed) instead of the bare substring. _(Quick; slightly less robust than a proper anchor.)_
- **C —** Leave as-is and require the banner to be untouched. _(Fragile — any banner edit silently produces a doubly-defined, stale module.)_

**Recommendation:** A — line-anchor the fallback search so `BLIND_EDGES` can never be matched.

**Decision:** Defer — one-shot migration already run; A (line-anchor the search) if re-run.

#### 69. Docstring extractor only accepts a triple-quoted string as the literal first token; a leading shebang/cookie/`from __future__` aborts migration

- **Location:** `tools/path_planner/editor/migrate.py:267`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** field data JSON migration
- **Traced path:** `run() -> _extract_module_docstring(source) -> startswith('"""'/"'''") fails -> raise _MigrationError -> run() returns _EXIT_ERROR (2)`

**What's wrong:** `_extract_module_docstring` skips only whitespace, then requires the next characters to be a
triple-quote opener; otherwise it raises `_MigrationError` and migration exits 2. If `field_data.py` begins with
anything before the docstring — a shebang, a `# -*- coding: utf-8 -*-` cookie, a license comment, or a
`from __future__ import annotations` — the function treats it as "no module docstring" and fails the whole
one-shot migration. The committed input starts directly with `"""`, so it works today, but the parser is brittle.

**Options:**
- **A —** Use `ast.parse` + `ast.get_docstring` (or skip comments/shebang/`__future__`/cookies) to locate the docstring robustly. _(Handles all leading-token variants; the proper fix.)_
- **B —** Skip a leading shebang/cookie/comment block before the triple-quote check. _(Covers the common cases; still hand-rolled.)_
- **C —** Leave as-is and document that field_data.py must start with the docstring. _(Constrains hand edits; brittle.)_

**Recommendation:** A — parse with `ast` so any valid module header migrates.

**Decision:** Defer — one-shot migration already run; A (parse with `ast`) if re-run.

#### 70. `loader.dump_field_json` docstring claims the migration creates the JSON parent dir, but `run()` never does

- **Location:** `tools/path_planner/editor/loader.py:179`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** field data JSON migration
- **Traced path:** `run() -> dump_field_json(json_payload, _FIELD_DATA_JSON) -> path.write_text(...) with no prior mkdir`

**What's wrong:** `dump_field_json`'s docstring states "The parent directory must already exist (the migration tool
creates it explicitly)." `run()` creates no directory before calling `dump_field_json`; it relies on the sibling
directory already existing. In the real repo layout it always does, so no failure occurs, but the documented
invariant is false and the implied safety net doesn't exist. If the dir were missing, `write_text` raises OSError,
caught at run() line 155 returning 2 — a clean failure, but not what the docstring promises.

**Options:**
- **A —** Make `run()` actually `mkdir(parents=True, exist_ok=True)` the parent before dumping, fulfilling the docstring. _(Cheap; makes the contract true.)_
- **B —** Fix the docstring to drop the false claim. _(Documentation-only; honest.)_
- **C —** Leave as-is. _(Misleading contract for a future reuser; no live failure.)_

**Recommendation:** B (or A) — either make the claim true or remove it; A is barely more work and safer.

**Decision:** Defer — one-shot migration already run; B (fix the docstring) if revisited.

### Offline — tests

#### 71. build_routes tests assert behavior against the live, editor-mutable `field_data.json`

- **Location:** `tools/path_planner/tests/test_build_routes.py:96`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** Python test suite (pytest)
- **Traced path:** `test_run_disconnected_node_warns_but_succeeds() -> _clone_field_data() (reads real field_data) -> build_routes._run() -> build_graph(BLIND_EDGES=[]) -> warns only if a node is blind-only-reachable`

**What's wrong:** `test_run_disconnected_node_warns_but_succeeds()` clones the *live* `field_data` module (not the
stable toy fixture), empties BLIND_EDGES, then asserts a `WARN:` line because some node becomes isolated. This only
holds because the current `field_data.json` has blind-only nodes (Start, finial). The toy fixture docstring calls
importing the live module "a regression bomb — every editor save would flip one or more tests red." It passes with
the current data, so it is not broken — the risk is latent: if the field is edited so every node is LINE-reachable,
emptying BLIND_EDGES isolates nothing and the assertion fails even though `_run()` still returns 0.

**Options:**
- **A —** Rewrite the test to use the stable toy fixture (or a purpose-built fixture with a guaranteed blind-only node) instead of the live module. _(Decouples the test from operator field edits; the fixture docstring already recommends this.)_
- **B —** Assert only on the return code (0) and not the WARN line, if the warning is incidental. _(Weakens coverage of the warning path.)_
- **C —** Leave as-is. _(A future field edit can turn this test red with no code regression.)_

**Recommendation:** A — base the test on a controlled fixture so it doesn't depend on the live field.

**Decision:** A — base the test on a controlled toy fixture (do before P3.5 so the field rewrite can't falsely break it).

#### 72. Golden `routes_toy.cpp` byte-equality couples the test to `pipeline_config.py` defaults

- **Location:** `tools/path_planner/tests/test_header_writer.py:167`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** Python test suite (pytest)
- **Traced path:** `test_routes_cpp_matches_golden() -> _emit_with_toy_field() -> emit_routes() bakes PIPELINE_CONFIG into routes.cpp -> compared byte-for-byte against frozen golden`

**What's wrong:** `emit_routes()` bakes the live `pipeline_config.PIPELINE_CONFIG` values into routes.cpp, and the
test compares emitted bytes against a frozen golden — so the golden must stay in lockstep with `pipeline_config.py`.
The toy source SHA256 and the PIPELINE_CONFIG default assertions match today, so it is consistent. The coupling is
partly mitigated (a pipeline_config default change would also break `test_pipeline_config_default_emit`, surfacing
the drift), but the full .cpp body has no independent parity check beyond the byte compare. A contributor changing
a default without regenerating the golden gets a byte-equality failure requiring an authorized rebaseline.

**Options:**
- **A —** Accept the coupling and document the "regenerate golden after a pipeline_config default change" workflow near the test. _(The byte-golden is intentional; just make the workflow explicit.)_
- **B —** Add a smaller, targeted assertion on the embedded PIPELINE_CONFIG values so a drift produces a precise message rather than a full-body byte diff. _(Better failure UX; some duplication with the golden.)_
- **C —** Leave as-is. _(Works today; a future default change yields a confusing byte diff.)_

**Recommendation:** A — document the rebaseline workflow; the golden coupling is by design.

**Decision:** A — document the regenerate-golden step (fires only on a rare pipeline-default change, not on point edits).

#### 73. No pytest config: several tests silently depend on warnings not being errors

- **Location:** `tools/path_planner/tests/test_visualizer.py:33`
- **Severity:** possible   **Scope:** pre-existing
- **Capability:** Python test suite (pytest)
- **Traced path:** `pyproject.toml has no [tool.pytest.ini_options] -> filterwarnings unset -> test_render_with_highlighted_path() renders without warnings.catch_warnings/pytest.warns`

**What's wrong:** `pyproject.toml` declares no `[tool.pytest.ini_options]` (no `filterwarnings`, `testpaths`, etc.).
Many tests rely on pytest's default of not escalating warnings to errors: `test_render_with_highlighted_path`
renders without suppressing/asserting warnings, and `build_graph`/visualizer emit `UserWarning`s for isolated
nodes. If `filterwarnings = error` were ever added, tests that emit incidental warnings without wrapping them in
`pytest.warns`/`catch_warnings` would fail. A configuration gap, not a current failure.

**Options:**
- **A —** Add a `[tool.pytest.ini_options]` block now (set `testpaths`, and either keep warnings non-fatal explicitly or wrap the emitting tests in `pytest.warns` and then enable `filterwarnings = error`). _(Hardens the suite deliberately and makes the dependency explicit.)_
- **B —** Wrap the known warning-emitting tests in `pytest.warns`/`catch_warnings` without changing global config. _(Targeted; leaves config absent.)_
- **C —** Leave as-is. _(Fine until someone hardens warnings; then several tests break unexpectedly.)_

**Recommendation:** A — add an explicit pytest config so the warning behavior is intentional, not accidental.

**Decision:** A — add an explicit `[tool.pytest.ini_options]` block.

---

## Review manually (uncertain)

The verifier could not confirm these. Each has a short explanation and a one-line "what to check." Where an
obvious fix exists it is noted, but most need a human to confirm the trigger before acting.

#### 74. Latched junction type is a single-tick sample; a transient misclassification during debounce is reported as the junction type

- **Location:** `WRO2026/Intersection.cpp:94`   **Severity:** possible   **Scope:** pre-existing
- **Capability:** intersection detection

The debounce streak gates *whether* a rising edge fires, not *which* type is latched. On the tick `_validStreak`
reaches `INTERSECTION_DEBOUNCE_TICKS` (2), `_lastType` is set from that tick's `raw`, whose `_classifyJunction`
verdict (LEFT/RIGHT/T_OR_CROSS) can vary tick-to-tick while the chassis straddles a junction. A fat T momentarily
reading LEFT on the crossing tick would latch LEFT and make PathTracker turn instead of pass through.

**What to check:** whether `_classifyJunction` is stable enough across the straddle window in practice, or whether a
majority/voted type over the streak (rather than the crossing-tick sample) is warranted. Fix if confirmed:
latch the most-common type over the debounce window rather than the single crossing-tick sample.

**Decision:** Verify in Phase 3 bench (B3.4) — see checklist; latch the majority type over the debounce window if confirmed.

#### 75. Host-compile harness is never compiled or run (no CI job, no local compiler) — latent breakage undetected

- **Location:** `WRO2026/test/CMakeLists.txt:20`   **Severity:** suspicious   **Scope:** PR-surface
- **Capability:** C++ host-compile unit test harness

The CMakeLists header and README both state the host-compile harness is deliberately not wired into CI, and
`ci.yml` has no ctest job; combined with "no host C++ compiler on this Windows machine," nothing in the normal
workflow compiles or runs these test executables. Any compile-level breakage (a mock drifting from a production
call site, an include-order regression) ships green because no gate exercises it.

**What to check:** whether wiring a host-compile + ctest job into CI is feasible/desired. If yes, add a CI job that
configures with cmake and runs ctest; that single change makes findings 73-79 actually enforced.

**Decision:** A — add a CI host-compile + ctest job (turns the whole C++ harness from decorative into enforced; no local compiler exists).

#### 76. WILL_FAIL overflow test passes on ANY non-zero build exit, not specifically the `static_assert`

- **Location:** `WRO2026/test/CMakeLists.txt:270`   **Severity:** possible   **Scope:** PR-surface
- **Capability:** C++ host-compile unit test harness

The TC4/AC9-negative test builds the overflow target and marks it `WILL_FAIL TRUE`, so ctest treats any non-zero
exit as a pass. The CMakeLists comment recognizes the risk and withholds `add_mock_force_includes`, but a stale
build dir, a missing generator, an unrelated `TasksTypes.h` edit breaking `sizeof/alignof` asserts, or a broken
Config.h would all fail the build for the wrong reason and still report green. The test can't distinguish "static_
assert fired" from "toolchain/config broke."

**What to check:** whether the build output can be matched against the expected `static_assert` message
(e.g. `PASS_REGULAR_EXPRESSION` on the diagnostic) so only the intended failure counts. Fix if desired: assert on
the expected compiler error text rather than just non-zero exit.

**Decision:** A — match the expected `static_assert` text (PASS_REGULAR_EXPRESSION) so only the intended failure counts.

#### 77. PathTracker `REVERSE_UNAVAILABLE` guard is hand-copied into the test, not the real code path — drift is silent

- **Location:** `WRO2026/test/test_path_tracker_bidirectional.cpp:137`   **Severity:** possible   **Scope:** PR-surface
- **Capability:** C++ host-compile unit test harness

TC12 asserts the REVERSE_UNAVAILABLE behavior against `replica_dispatchEdgeAfterTurnGuard`, a ~15-line hand-copy of
the guard that lives in `PathTracker.cpp::_dispatchEdgeAfterTurn`. Unlike the direction logic (promoted to the
shared `DirectionPolicy.h`), this guard is a manual replica whose only drift protection is a `// MIRROR:` comment.
If production's guard gains a condition, the replica keeps passing against stale logic.

**What to check:** whether the guard can be promoted to a shared header/function (like `resolveDirection` was) so
test and production link the same symbol. Fix if feasible: extract the guard so it isn't duplicated.

**Decision:** A — promote the guard to a shared header so test and production link the same symbol.

#### 78. Mirror-based Tasks/Mission tests assert against an inline reimplementation, never the real `Tasks.cpp`/`Mission.cpp`

- **Location:** `WRO2026/test/test_tasks_dispatch.cpp:195`   **Severity:** possible   **Scope:** PR-surface
- **Capability:** C++ host-compile unit test harness

All of TC1..TC15 exercise `TasksMirror`/`MissionMirror` — full inline reimplementations of the registry/dispatch/
RAII-guard and the Mission TASK-case FSM — rather than the real sources. Only the POD layout (from the real
`TasksTypes.h`) is bound to production. The behavioral assertions (reentrancy guard, abort ordering, queue-full
rejection, unknown-name handling, yield) all run against the mirror, so a logic regression in the actual code is
invisible. A documented design choice, but the file's title overstates what is verified.

**What to check:** whether the tests can link the real `Tasks.cpp`/`Mission.cpp` (with mocked hardware) instead of
mirrors, so the dispatch behavior is verified end-to-end. This is the most impactful of the harness items: the
"framework dispatch is tested" claim rests on a copy.

**Decision:** A — link the real `Tasks.cpp`/`Mission.cpp` against the mocks (pairs with #75; biggest harness payoff).

#### 79. Union write-bytes/read-typed punning in TC3 relies on UB and a little-endian host

- **Location:** `WRO2026/test/test_tasks_dispatch.cpp:865`   **Severity:** possible   **Scope:** PR-surface
- **Capability:** C++ host-compile unit test harness

TC3 packs four `uint32_t` through `TaskArgs::bytes` (memcpy) then reads them via `TaskArgs::u32`, asserting
`u32[0..3] == 1,2,3,4`. `TasksTypes.h`'s own docstring flags this write-one-member/read-another pattern as
`[class.union]/4` UB that only works under the GCC/clang union-aliasing extension, and the assertion assumes
little-endian byte order. On a big-endian or strict-aliasing host the test would fail/be UB even though production
firmware (avr-g++/Cortex-M4) is fine — non-portable across the "any C++17 host" the harness advertises.

**What to check:** whether the harness is only ever run on little-endian GCC/clang (then it's a non-issue worth a
comment), or should be made portable. Fix if portability matters: read back via the same member, or memcpy out.

**Decision:** C — comment that the harness assumes a little-endian GCC/clang host.

#### 80. Bidirectional test compiles against `routes.h` externs whose definitions are never linked

- **Location:** `WRO2026/test/CMakeLists.txt:184`   **Severity:** possible   **Scope:** PR-surface
- **Capability:** C++ host-compile unit test harness

`test_path_tracker_bidirectional` includes `routes.h` (via DirectionPolicy.h/PathTracker.h/FieldMap.h), which
declares `extern` NODES/NEXT_HOP/EDGES/PIPELINE_CONFIG, but `routes.cpp` is deliberately not in the source list.
The build only succeeds because the current test body odr-uses none of those externs. The moment a future test
calls `FieldMap::getNextHop/getEdge/getNode` or reads PIPELINE_CONFIG for a fixture, the link fails with
undefined-reference errors unrelated to the logic under test.

**What to check:** whether `routes.cpp` (or a toy routes definition) should be added to the bidirectional target's
sources pre-emptively, so a future fixture that touches real graph data links cleanly.

**Decision:** A — add `routes.cpp` (or a toy routes def) to the bidirectional target so future fixtures link cleanly.

#### 81. Turn profile runs away / never recovers on overshoot past target

- **Location:** `WRO2026/Drive.cpp:1019`   **Severity:** bug *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** drive control

`_turnDeltaSign` is captured once at dispatch and never recomputed. `_updateTurnProfile` only transitions to BRAKE
when `abs(rawDelta) <= TURN_TOLERANCE_DEG` (1°). If the chassis overshoots so the signed error flips sign and
`|rawDelta| > 1°`, the code still drives the wheels with the original fixed sign — rotating *away* from target —
and BRAKE can only be reached by passing back through the 1° window. With SLOW PWM=20 (needed to break stiction)
and a 1° tolerance, a single-tick overshoot >1° could drive further past every tick until the ~10 s drain-timeout
fault. The verifier could not confirm whether the closed-loop turn (SYNC/GYRO) physically overshoots this way.

**What to check:** on hardware (or a faithful sim), whether a real turn overshoots the 1° window and, if so, whether
the wheel sign keeps it rotating away. If confirmed, this is a high-priority runaway. Likely fix: recompute the
drive sign each tick from the current signed error (or brake once `|rawDelta|` stops decreasing), rather than
latching `_turnDeltaSign` at dispatch.

**Decision:** Verify in Phase 3 bench (B3.2) — see checklist; recompute the drive sign each tick if turns overshoot the window.

#### 82. Blocking move/turn issued while a fault is latched hangs ~10 s then re-faults and drops the command

- **Location:** `WRO2026/Drive.h:1308`   **Severity:** bug *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** drive control

When `_fault.kind != NONE`, `_dispatchNext()` returns immediately (fault gate). A sync-blocking builder destructor
unconditionally calls `_drainBlocking()` after enqueueing, but the queued command can never dispatch while the
fault is latched, so `_qCount` stays > 0 and the drain spins until its `(qCount+1)*10000 ms` deadline, then
hard-stops, overwrites the fault with `DRAIN_TIMEOUT`, and clears the queue — silently dropping the new command.
Neither the destructor nor `_drainBlocking` checks `hasFault()` before draining.

**What to check:** confirm a blocking `driveDist`/`turnDeg` issued after a latched fault (without `clearFault()`)
actually blocks ~10 s and drops the command. Likely fix: have the blocking path check `hasFault()` up front and
return immediately (preserving the original fault) instead of draining.

**Decision:** Verify in Phase 3 bench (P3.1 fault path) — see checklist; check `hasFault()` up front and return immediately if confirmed.

#### 83. Profiled move with cruise PWM 0 never completes and hangs until drain timeout

- **Location:** `WRO2026/Drive.cpp:633`   **Severity:** suspicious *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** drive control

If the resolved cruise PWM is 0 (e.g. `.setPower(0)` on a profiled distance move ≥ `TSHAPE_MIN_MOVE_CM`),
`_profilePeakSpd` resolves to 0. After ramping to 0 the per-tick setpoint is 0, the encoder never advances, `frac`
never reaches 1.0, and the profile state machine never completes. The only exit is the ~10 s blocking-drain
timeout (then fault) or, async, a permanently stalled queue. The distance path has no `cruise==0` guard (the time
variant has a `cmToMs` fallback that still doesn't complete the move).

**What to check:** whether any real call path can resolve cruise to 0 (a preset/typo). Likely fix: reject
`cruise <= 0` at the profile builder, or complete the move immediately when cruise resolves to 0.

**Decision:** Verify in Phase 3 bench (B3.1) — see checklist; reject `cruise <= 0` at the profile builder if a real path can resolve it.

#### 84. Profile completion keyed only on left encoder (M2); right-driven slip/stall hangs the move

- **Location:** `WRO2026/Drive.cpp:620`   **Severity:** possible *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** drive control

The distance-profile completion `frac` is computed exclusively from the left motor encoder (M2). If the left wheel
slips/stalls or its encoder reads short while the chassis covers ground via the right wheel, `frac` never reaches
1.0 and the move never completes. For RAW profiled moves with bias, the single-encoder assumption ignores the
right wheel entirely. (Note this overlaps finding #20, which is about the ramp-up measure specifically.)

**What to check:** whether SYNC/GYRO balancing makes a left-only measure safe in practice, or whether completion
should use the both-wheel average (consistent with the distance gate and #20's recommendation).

**Decision:** Verify in Phase 3 bench (B3.1) — see checklist; switch completion to the both-wheel average (with #20) if a move hangs on wheel slip.

#### 85. `deadbandSnap` applied to SYNC/GYRO profile setpoints forces a minimum closed-loop command

- **Location:** `WRO2026/Drive.cpp:685`   **Severity:** possible *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** drive control

For SYNC/GYRO profiled moves the per-tick setpoint is passed through `deadbandSnap`, which snaps any non-zero
magnitude below `PWM_MIN_MAGNITUDE` (20) up to 20. The documented contract is that SYNC/GYRO close the loop on the
STM32 and should not have feedforward shaping (which is why `_applyBias` is skipped on these paths). Forcing a
floor of 20 means the STM32's controller can't command a small terminal speed; the ramp-down to endSpeed=0 jumps
20→0 rather than fading, and any sub-20 cruise is silently raised — contradicting the module's own rationale.

**What to check:** whether `deadbandSnap` should be skipped (or replaced with a true zero pass-through) on the
SYNC/GYRO profile setpoints, the way `_applyBias` already is. Likely fix: don't deadband-snap closed-loop setpoints.

**Decision:** Verify in Phase 3 bench (B3.1/B3.2) — see checklist; skip deadbandSnap on closed-loop setpoints if it floors the terminal speed.

#### 86. `calibrateMotorBias` reads per-motor `getDegrees()` but resets via `DriveDC.resetCounter()` — reset/read scope may not match

- **Location:** `WRO2026/Drive.cpp:460`   **Severity:** possible *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** drive control

`calibrateMotorBias` and `_execProfileMove` read `M2/M3.getDegrees()` for per-wheel deltas, but zero with
`MiniR4.DriveDC.resetCounter()`, which issues `Set_Drive_Reset_Count` against the DriveDC instance id, not the
individual M2/M3 ids (`resetEnc()` has the same pattern). If the lower-MCU's DriveDC reset doesn't zero the same
counters `M2/M3.getDegrees()` read, post-spin readings aren't clean deltas and the computed bias scalars are
offset by residual counts.

**What to check:** confirm against the MMLower/DriveDC firmware whether `DriveDC.resetCounter()` zeroes the same
counters `M2/M3.getDegrees()` reads. If not, reset via the individual motor counters instead. This affects every
raw-PWM move's bias, so it's worth verifying at bring-up.

**Decision:** Verify in Phase 3 bench (P3.2/B3.2) — see checklist; reset per-motor counters if the DriveDC reset scope mismatches. Affects every raw-PWM bias.

#### 87. Default/uncalibrated row (LOW=0, HIGH=9) reports false on-line, contradicting the Config doc and bypassing the dead-channel warning

- **Location:** `WRO2026/Config.h:345`   **Severity:** suspicious *(unconfirmed)*   **Scope:** PR-surface
- **Capability:** 8-channel reflectance line array sensing

Config.h:345-346 says the zeroed `LINE_CALIB_LOW` defaults cause every channel to read fully-bright = off-line.
That appears false: with LOW=0/HIGH=9, `_isOnlineFromCache` computes `norm=(raw-0)/(9-0)`, so any channel `raw >= 5`
(norm ≥ 0.556 > 0.5) is declared ON. An uncalibrated board on white (which reads non-zero packed values) could
report on-line with a bogus error, instead of failing safe to off-line. Worse, the `begin()` guard (LOW>=HIGH) is
`0>=9` false, so it does *not* flag the default row as uncalibrated — no boot warning.

**What to check:** what raw packed values an uncalibrated board actually reads on white (are they ≥ 5 per channel?).
If yes, the "safe default" comment is wrong and either the default LOW/HIGH should fail-safe to off-line or `begin()`
should warn that calibration was skipped. PR-surface (Config.h) — worth confirming before competition.

**Decision:** Verify in Phase 3 bench (B3.3) — see checklist; fail-safe the default to off-line or warn at `begin()` if an uncalibrated board reads on-line.

#### 88. `begin()` proceeds to use a failed FRONT board's readings; failure is only surfaced via the sketch boot gate

- **Location:** `WRO2026/WROLineArray.cpp:95`   **Severity:** possible *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** 8-channel reflectance line array sensing

When the front board's `MatrixColor::begin()` returns false, `begin()` sets `frontOk=false` and returns false,
relying entirely on the sketch's `_setupFailed` boot gate to halt. There is no internal "initialised" flag: if a
caller invokes the getters without honoring the false return, `_readBoardInternal` still issues I2C to the
uninitialised board (getColor returns 0xFF → caught → `_lastOnline=false` every tick). Combined with #22, a failed
`begin()` degrades to "permanently lost line" rather than a hard stop if the boot gate is bypassed.

**What to check:** whether any path (a future diagnostic sketch, a refactor) could call the getters without gating
on `begin()`'s return. If that's a real risk, add an internal initialised flag so the getters refuse after a failed
`begin()`. Latent defense-in-depth.

**Decision:** Verify in Phase 3 bench (P3.3) — see checklist; add an internal "initialised" flag if a getter-without-begin path is real.

#### 89. `_classifyJunction` LEFT/RIGHT predicates can misbehave if junction index constants exceed 7

- **Location:** `WRO2026/WROLineArray.cpp:863`   **Severity:** possible *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** 8-channel reflectance line array sensing

The junction predicates trust `LINE_JUNCTION_LEFT_HI` and `LINE_JUNCTION_RIGHT_LO` to be in [0..7], but there is no
`static_assert` bounding them (unlike `LINE_BOARD_COUNT` and the I2C ports, which do have static_asserts). A
misconfigured `RIGHT_LO==8` makes the right-lit loop never execute (ok stays true) so RIGHT can fire on a totally
blank bar; an out-of-range `LEFT_HI` silently changes which channels count as the left half.

**What to check:** add `static_assert(LINE_JUNCTION_LEFT_HI < 8 && LINE_JUNCTION_RIGHT_LO < 8 && ...)` near the
other junction constants so a fat-fingered tuning value fails to compile rather than emitting spurious verdicts.
(This is a clear, cheap fix even though the trigger requires a misconfig.)

**Decision:** Defer — cheap, but no urgency; the trigger requires a tuning misconfig.

#### 90. Stale `_passThrough` after a turn-required replan makes the next BLIND edge start its trapezoid at cruise from a standstill

- **Location:** `WRO2026/PathTracker.cpp:792`   **Severity:** suspicious *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** graph-based autonomous navigation (PathTracker)

This is the same defect as confirmed finding **#32** (PathTracker.cpp:803), described from a slightly different line
(792). The pass-through flag set in the LINE_FOLLOW peek is not cleared when `_onPlanNextHop()` decides a turn is
required, so the post-turn BLIND dispatch reads stale `_passThrough==true` and starts the trapezoid at cruise PWM
from rest. It lands in the uncertain bucket here, but the confirmed entry #32 captures it.

**What to check:** resolve together with **#32** — the recommended fix (clear `_passThrough` on the turn-required
branch) covers both.

**Decision:** A — same fix as #32 (clear `_passThrough` on the turn-required branch).

#### 91. `goTo()` with a bad node id or before zeroing overwrites and erases a pre-existing latched fault

- **Location:** `WRO2026/PathTracker.cpp:316`   **Severity:** possible *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** graph-based autonomous navigation (PathTracker)

`_applyGoToBuilder()` checks goal-id validity (316) and `Localization.isZeroed()` (321) and calls `_enterFault(...)`
in those branches *before* the `if (_fault.kind != NONE)` no-op guard at line 326. So if PathTracker is already
latched (e.g. UNREACHABLE with diagnostic from/to nodes), a later `goTo(badId)` or a `goTo()` while not zeroed
re-enters `_enterFault`, overwriting `_fault.kind`/`fromNode`/`destNode` and destroying the original diagnostic —
defeating the "goTo while faulted → no-op" intent. No motion misbehavior, but the latched fault the operator is
reading gets corrupted.

**What to check:** whether the validity/zeroed checks should sit *after* the `_fault.kind != NONE` guard so an
existing fault is preserved. Likely fix: move the no-op fault guard to the top of `_applyGoToBuilder`.

**Decision:** A — move the no-op fault guard to the top of `_applyGoToBuilder` so an existing latched fault is preserved.

#### 92. Async `goTo` has no watchdog: a never-completing turn or never-detected intersection hangs the FSM forever

- **Location:** `WRO2026/PathTracker.cpp:433`   **Severity:** possible *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** graph-based autonomous navigation (PathTracker)

The drain-timeout (`DRAIN_TIMEOUT`) only exists on the sync-blocking path. For an async `goTo` (the path Mission
uses), the FSM has no time budget: in TURNING it blocks indefinitely on `Drive.isIdle()` (an async turn that
physically stalls never sets `Drive.hasFault()`), and LINE_FOLLOW waits forever on `Intersection.isAtIntersection()`
if the marker is missed. Mission's GOTO ACTIVE only polls `PathTracker.isIdle()` with no per-step timeout, so the
whole mission can stall silently with no fault/beep. The only escape is an external `abort()`.

**What to check:** whether a per-edge/per-step watchdog (time or distance budget) should be added to the async path
so a wedged turn or missed marker faults instead of hanging. This is a real-run failure mode worth a deliberate
design decision; verify the stall conditions, then add a watchdog if accepted.

**Decision:** C — leave as abort-only for now; a hung async step is stopped only by an external `abort()`. Accepted known risk.

#### 93. A blocking `goTo`'s drain spin absorbs goals enqueued concurrently by `Mission.update()` and over-blocks

- **Location:** `WRO2026/PathTracker.cpp:360`   **Severity:** possible *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** graph-based autonomous navigation (PathTracker)

`_drainBlocking()` spins until `_qCount == 0 && _state == IDLE`, ticking `cooperativeTick()` each pass, which runs
`Mission.update()`. If a Mission is RUNNING during a hand-written blocking `goTo`, Mission's GOTO step issues
`PathTracker.goTo(...).async()` mid-drain, enqueuing another goal. Because the drain exits only on queue-empty AND
IDLE, the blocking call now also drives the Mission's goals to completion before returning — extending the
supposedly single-goal call and possibly tripping `DRAIN_TIMEOUT` (the budget was computed once at entry).

**What to check:** whether mixing a hand-written blocking `goTo` with a running Mission is a supported usage. If it
is, the drain exit condition should be scoped to the originally-enqueued goal, not "queue empty." Likely a
bench-only edge case — confirm the usage matters before changing the drain semantics.

**Decision:** Verify in Phase 3 bench (bench-only) — see checklist; scope the drain exit to the originally-enqueued goal if mixing blocking goTo + Mission matters.

#### 94. Two pivot edges sharing one following edge double-apply compensation

- **Location:** `tools/path_planner/planner.py:230`   **Severity:** possible *(unconfirmed)*   **Scope:** pre-existing
- **Capability:** all-pairs Dijkstra planner

`apply_pivot_compensations` reads `next_edge_data['distance_cm']` live (230) and writes back the shortened value
(237). If two distinct pivot edges resolve to the *same* following edge, the second computes its reduction off the
first's already-shortened distance, compounding the subtraction. In the real `compute_all_pairs_next_hop` flow this
is structurally prevented (bidirectional edges trip the "exactly one forward out-edge" guard at 214), but the
function is public and reusable, so a hand-built directed graph where `dst` has one forward out-edge but is entered
by two pivots from non-adjacent sources slips past the guard. Note: pending finding #42, the compensated distance
never reaches the firmware at all, so this only matters once #42 is fixed.

**What to check:** only relevant after #42 is resolved (so compensation actually affects output) and only on
hand-built directed graphs. If both hold, guard against applying compensation to an already-compensated following
edge (mark edges as compensated).

**Decision:** A — guard against double-applying pivot compensation (mark edges compensated); relevant now that #42 lands.

---

## Phase-3 bench verification checklist

These findings have a real mechanism but a trigger only the real chassis can confirm or kill. Per the
triage decision they are **not fixed in code now** — each is folded into the Phase-3 bench plan and verified
at the benchmark noted. If the bench confirms the trigger, apply the fix on the right (also recorded on each
finding's Decision line).

| # | What to watch for | Bench / task | Fix if confirmed |
|---|---|---|---|
| 81 | A turn that overshoots the 1° window keeps rotating away until the ~10 s drain fault | B3.2 (turnDeg ×20) | recompute the drive sign each tick from the current signed error |
| 82 | A blocking move/turn issued without `clearFault()` blocks ~10 s then drops the command | P3.1 fault-path bench | check `hasFault()` up front, return immediately |
| 83 | A profiled move whose cruise resolves to 0 never completes (hangs to drain timeout) | B3.1 | reject `cruise <= 0` at the profile builder |
| 84 | Left-wheel slip/stall hangs a distance-profile move (`frac` never reaches 1.0) | B3.1 | use the both-wheel average (with #20) |
| 85 | Closed-loop terminal speed floored at 20; ramp-down jumps 20→0 instead of fading | B3.1 / B3.2 | skip `deadbandSnap` on closed-loop setpoints |
| 86 | `DriveDC.resetCounter()` may not zero the counters `M2/M3.getDegrees()` read → biased deltas | P3.2 / B3.2 | reset via the individual motor counters |
| 87 | An uncalibrated board on white may report on-line (norm ≥ 0.5) with no boot warning | B3.3 / calibration | fail-safe the default to off-line, or warn at `begin()` |
| 88 | Getters issue I2C to an uninitialised board if the boot gate is bypassed | P3.3 | internal "initialised" flag |
| 74 | A fat T momentarily reading LEFT on the crossing tick latches LEFT → wrong turn | B3.4 (intersection ×20) | latch the majority type over the debounce window |
| 93 | A blocking `goTo`'s drain absorbs Mission-enqueued goals and over-blocks | bench only | scope the drain exit to the originally-enqueued goal |
