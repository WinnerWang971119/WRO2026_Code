# WRO 2026 Mosaic Masters — Design Decisions

## Context

This is the WRO RoboMission Senior 2026 challenge ("Mosaic Masters"). The robot must autonomously navigate a field marked with black lines and waypoints, deliver three tools to specific zones, place colored mosaic tiles into a randomly-patterned 3×3 frame, and deliver colored cement blocks to matching deposit areas — all within a 2-minute run, scoring up to 233 points.

The hardware is a Matrix R4 robot kit. The controller is a Matrix Mini R4, which is an Arduino UNO R4 WiFi paired with an STM32 co-processor that handles motor control and the 6-axis IMU at the firmware level. Programming is C++ via the Arduino IDE.

## Why we abandoned wall-following and dead-reckoning

The first instinct on a competition field is often time-of-flight wall sensors plus odometry. We rejected this approach early. The field has too many obstacles and zone boundaries that confuse distance sensors, and pure dead-reckoning accumulates error too quickly across a 2-minute run. Encoder slip, wheel drift, and a single missed turn all compound silently until the robot is somewhere it doesn't know about.

Instead, the field's existing black lines and intersections become the primary sensing substrate. Lines are reliable, fixed, and unambiguous. Intersections are discrete events the robot can use as ground-truth checkpoints — every time it crosses one, its understanding of "where am I" gets refreshed against reality.

## The three-layer mental model

The system is organized around three conceptual layers, each consuming the abstraction of the layer below it.

The **perception layer** turns raw sensor readings into discrete events: "I'm centered on a line" or "I just arrived at an intersection." Above that lies the **map layer**, a hand-built graph where nodes are intersections and edges are line segments labeled with cardinal directions. The **planning layer** runs Dijkstra over this graph to produce node sequences, then translates those sequences into low-level commands — turn left at this node, drive straight through that one — based on the robot's current heading.

This separation is the core design principle. Task-level logic ("go grab the tool, drive it to the parking lot, drop it") never touches motors directly; it composes navigation primitives. A new task is a few lines of orchestration, not a new movement sequence. This is what makes the system feel programmable instead of scripted.

## Why Arduino reshapes everything

An earlier draft of this architecture borrowed Python conventions — abstract base classes, dependency injection, hardware abstraction layers. Those are wrong for this platform. The Matrix R4 runs C++ on a microcontroller with limited RAM, no operating system, and a single-threaded `setup()`/`loop()` execution model. Arduino code that tries to look like a Python service ends up either failing to compile, fragmenting memory, or blocking the main loop in ways that break real-time response.

The architecture therefore uses **modules** (paired `.h`/`.cpp` files) rather than class hierarchies, **global singletons referenced through `extern`** rather than dependency injection, and **fixed-size arrays** rather than `std::vector` or `std::map`. Every module that has work to do across multiple loop iterations exposes an `update()` method that returns immediately — no `delay()` calls, no `while` loops waiting on hardware. Time is tracked with `millis()` comparisons.

This is not stylistic. On Arduino, the alternatives literally do not work.

## The PID question — where it lives, where it doesn't

A common mistake is putting motor PID control in the application code. The Matrix R4's STM32 co-processor handles low-level motor control internally: when the application says "set the left motor to 200 RPM," the STM32 closes the loop on the encoder feedback by itself. The application never sees PWM duty cycles or raw encoder counts in the inner control loop.

This means PID for **motor speed and position is not our concern**. It's solved by firmware below us. What *is* our concern is the **line-following PID**: the controller that converts "the line is offset 3 cm to the right of the sensor array" into "left wheel target speed = 220, right wheel target speed = 180." This PID lives in the application code and outputs target speeds, which the motor library then realizes precisely through its own internal loop.

The exact name of the Matrix library's motor API, and whether it provides any line-tracker helper, needs to be confirmed by reading the official examples directly. Documentation search was inconclusive, and this is one of the open items to verify before coding starts.

## Localization as a single source of truth

Only one module is allowed to answer the question "where am I?" — the Localization module. It holds the current node and current heading. Every other module reads from it; only the navigation primitives write to it. When the line-follower detects an intersection event, it informs Localization, which advances the node. When the drive base completes a turn, it informs Localization, which updates the heading.

This discipline matters because as the codebase grows, the temptation to "just keep a local copy" of the position becomes enormous, and that's exactly how silent inconsistencies enter a system. If anything ever gets confused about position, there is exactly one place to look and exactly one place to fix.

## Mosaic pattern reading via camera

The frame's color pattern is randomized each round by a printed paper underneath. Reading this pattern is essential — without it, the 120-point mosaic placement task is unreachable. The chosen approach is camera-based: the robot positions itself in front of the frame, captures one image, samples small pixel regions at the center of each of the 9 cells, averages the RGB values, and classifies each cell against pre-calibrated baselines for yellow, blue, green, and white.

Two risks deserve attention. Lighting changes between practice and competition venues will shift the RGB values, so the color baselines must be recalibrated on-site before each run. And the camera's position relative to the frame must come from line-following navigation, not from odometry, because a few millimeters of drift would put the sample points on the wrong cells. The robot approaches the frame along a known line and stops at a known intersection; the camera-to-cell geometry is then fixed by construction.

## Manipulator deferred to control primitives only

The mechanical design for picking up tools, placing tiles, and handling cement blocks is not yet finalized, and the three object types differ significantly in shape and weight. Rather than designing high-level actions like `pickTool()` prematurely, the manipulator module currently exposes only low-level control: set the lift to a height, set the gripper to an angle, set the arm to an angle. Once the mechanism is built and tested, those primitives will be composed into named actions — but committing to those names now would invite redesign later.

This is a deliberate "decide as late as possible" choice for the part of the system whose hardware is least settled.

## Time scheduling and the 2-minute reality

The maximum score is 233, but no team realistically completes every task in 2 minutes. The mission scheduler treats this as an explicit optimization problem: each task carries an estimated duration and an expected score, and a greedy "score per second" ordering decides what to attempt and in what order.

The mosaic pattern read happens first because it determines which tile colors will be needed downstream. Cement delivery is high-value-per-second because pushing piles is faster than picking individual pieces. Tool delivery is moderate — three independent 15-point opportunities. The brick trowel returns to the start zone for free at the end, since the robot is heading there anyway.

Every task carries a maximum-time guard. If a task overruns, the scheduler aborts it and moves on. With about 20 seconds left, the robot abandons whatever it's doing and executes a return-to-start, both to grab the brick-trowel-in-start-zone score and to avoid finishing in some random location.

## Failure recovery as a first-class citizen

Three failure modes are anticipated and handled explicitly. **Lost line**: the line-follower stops seeing readings consistent with a line. The robot pauses, sweeps left and right slowly, and resumes if the line is reacquired. **Missed intersection**: too much time has passed since the last expected node. The robot reverses a short distance and re-runs the intersection detector. **Failed grip**: the manipulator reports it didn't successfully pick up its target. The task retries once, then yields to the next task.

These recoveries are deliberately not heroic. They are simple, bounded, and cheap. The principle is that one task failing should never cascade into the rest of the run failing.

## What is deliberately not in scope right now

There is no logging system in this iteration, and no SD card. Debug visibility is limited to whatever the OLED screen and Serial console can show during testing. This is a deliberate simplification. Full data logging and replay tooling can be added later if it proves needed, but the upfront cost of doing it correctly — making every module's inputs deterministic and recordable — is high enough that it would slow down the more important work of getting basic navigation working. If something goes wrong in competition, the team will rely on observation and OLED-displayed event traces rather than post-hoc data analysis.

This is a trade we are choosing with eyes open. If it bites us, it bites us; the cost of fixing it later is much smaller than the cost of building it now while everything else is still moving.

## Motor-bias calibration (Strategy B); Strategy C on the shelf

The TT-motors driving the chassis are not matched. Two units of the same part number can differ by 10–20% in RPM at the same PWM duty cycle, and that asymmetry is exactly the kind of thing that wrecks an outer P+D line-following loop: the controller ends up chasing both the line error and the mechanical bias of the motors, and the two confound each other into oscillation. Phase 1's line-follower runs `Drive.setSpeed(L, R)` every tick — which routes to MatrixMiniR4's `Move` (open-loop PWM) — so any motor-pair mismatch shows up directly at the wheels.

The chosen fix is **Strategy B**: a static, calibrated bias scalar `_biasL` / `_biasR` applied inside `Drive` to every raw-PWM path (`setSpeed`, `driveDist`, `driveTime`). Calibration is a manual one-shot — `Drive.calibrateMotorBias()` spins both motors for ~2 s wheels-free, reads the encoder deltas, and prints two scalars to Serial. Diego hand-edits `Config::MOTOR_BIAS_L` / `MOTOR_BIAS_R` and re-flashes. From then on, `Drive.begin()` propagates those values to the runtime members. There is no EEPROM persistence and no auto-recalibration — the source file is the single source of truth, which keeps the firmware deterministic and reviewable.

The Sync, Gyro and Turn paths deliberately do **not** apply the bias. Those paths close the loop on the STM32, and stacking a feedforward scalar on top of an inner PID would fight the regulator instead of helping it. Each of those methods carries a one-line `// No _applyBias here — STM32 closes the loop.` comment so the discipline is visible at the call site.

We also looked hard at three alternatives and chose to defer all of them. **Strategy A** would have switched the line-follower from raw `Move` to `MoveSync` and let the lower-MCU PID handle wheel matching. Reading `MMLower::Set_Drive_MoveSync_Func` (MMLower.cpp:1725) shows the payload to the STM32 is only `(power_L, power_R)` plus a motor-pair id — five bytes, no per-motor rate target. The single 3-gain PID slot exposed by `setMoveSyncPID` most plausibly drives `error = rate_L - rate_R` toward zero, which would force shared encoder rates and cancel the asymmetric L:R mix the outer P+D produces. The source alone cannot prove ratio-preserving versus shared-rate semantics, but the asymmetric `MoveSync(60, 40)` case is never used in a tight loop in any vendor example, and a five-second hardware test would resolve it. Until that test, A is treated as dead.

**Strategy C** is an R4-side encoder-rate PID running on top of the bias-corrected commands. The bulk-read API `MMLower::GetALLEncoderSpeed` (MMLower.h:391) already exists, so the cost is roughly one extra UART round-trip per tick — cheap if it turns out to be needed. We are not building it now because YAGNI: B handles the dominant term (static manufacturer mismatch); battery-sag and friction drift are smaller, higher-frequency disturbances that will only matter once the chassis is on the actual track and we can see whether residual oscillation is still a problem. C slots in cleanly on top of B if it is.

**Strategy D** would have cascaded the gyro into the line-following control law itself, which restructures everything and is too invasive for what is fundamentally a motor-matching problem. It stays on the shelf in case both B and C fail.

The trigger to revisit this is the first hardware track test. If the chassis still oscillates after `calibrateMotorBias()` produces good scalars, we layer C on top. If C is also insufficient, we escalate to D. The touchpoints in code are `Config::MOTOR_BIAS_*`, `Drive::_applyBias` and the calibration routine, and a `@note Strategy B` comment everywhere a raw-PWM path is biased; `LineFollower::update` references this decision in its docstring.

## Fluent / builder API for any verb with 3+ knobs

Years of writing this style of code has shown that wide positional argument lists like `moveDist(power, cm, brake, mode, blocking, heading)` become unreadable at the call site within weeks — every parameter has to be looked up, and every reordering breaks every caller. The Drive layer alone could easily grow to half a dozen mutually-exclusive `driveDistSync` / `driveDistGyro` / `driveDistGyroBlocking` variants that all duplicate doxygen and all hide the actual differentiator behind a method name suffix.

Phase 1.5 retrofits every Drive verb with 3 or more knobs into a fluent builder pattern. `Drive.driveDist(30)` returns a `[[nodiscard]]`-marked builder; chained `.setPower(50).setMode(DriveMode::GYRO).setBrake(false).setBlocking(true)` setters each return a reference to the builder; the move actually runs when the temporary destructs at end-of-statement (RAII auto-execute). The compiler warning fires if anyone stores the temp without using it. Copy is deleted; move is implemented with an `_executed` flag transfer so the destructor is idempotent in the rare case it ever runs after a move. `LineFollower.setEnabled(bool)` follows the same pattern with `.setBrake(bool).setPreset(Config::SpeedPreset)`.

The continuous primitives `setSpeed`, `setSpeedSync`, `forwardGyro`, and `stop` deliberately stay direct (non-fluent) because they take 0–2 trivial arguments — fluent there would be ceremony. The rule is **3+ knobs → builder; 0–2 knobs → direct call**, and it applies project-wide, not just to Drive. Future modules (path planner, mission scheduler, manipulator command interface) inherit this convention.

The retrofit collapsed eight previously-separate Drive methods into four fluent verbs. `driveDist(power, cm)`, `driveDistSync(power, cm)`, `driveDistGyro(power, cm)` all became `driveDist(cm).setMode(DriveMode::RAW|SYNC|GYRO)`; the same collapse happened for `driveTime*` and the four turn variants (sync/blocking variants became `.setBlocking(true)`). The eight legacy public signatures are gone from `Drive.h` — they are no longer reachable from caller code. The actual MMLower-call logic moved into private `_execXxx` methods that the builders' destructors dispatch to; the `_applyBias` discipline (RAW only) and the C1 turn-fail unblock contract carry forward verbatim.

One reviewer-flagged race needed a guard. The C1 fix's "set `_turnDone = true` immediately on a non-zero TurnGyro rc" was unconditional in the first cut, which would clobber the done-flag if a *prior* bounded task were still in flight when a *new* failed turn was issued. The fix (W1 guard) snapshots `_turnDone` at entry and only writes it back to `true` in the failure branch when the slot was actually idle — preserving both the C1 unblock guarantee for a fresh failed turn and the prior task's pending state when one exists. Failed turns also now print their rc to Serial during bring-up so silent failures surface.

## LineFollower disable does not imply brake; the planner owns end-of-segment behaviour

`LineFollower.setEnabled(false)` used to actively brake by calling `Drive.stop(true)` unconditionally. That made line-follower disable "safe" in the sense that the chassis stops, but it also made it useless for one of the most common moves the path planner needs: drop the line and coast a little further before turning, so there is room to swing into the next leg. The planner had no way to ask for "stop following but keep rolling".

The new contract: `LineFollower.setEnabled(false)` clears `_enabled`, zeroes `_lastErr`, and clears `_lostLine` — that part is invariant. Whether the chassis actively brakes is the **planner's** decision, expressed through `.setBrake(bool)` on the EnableBuilder. The default is `true` (braking is the safe choice when no caller specifies), but `LineFollower.setEnabled(false).setBrake(false)` lets the planner take responsibility for the next move. If the planner doesn't issue a follow-up command, the chassis simply continues at whatever speeds were last latched on `Drive` — which is exactly the behaviour the coast-into-corner case wants.

The brake parameter is silently ignored on `setEnabled(true)` because there is no "stop" to apply when entering an enabled state. The doxygen on `setBrake` documents this so callers don't expect it to do anything when chained after `setEnabled(true)`.

The principle generalises beyond LineFollower: any module that "turns off" should not implicitly issue actuator commands on behalf of the caller. The next layer up is the right place to decide what comes next.

## Trapezoidal velocity profile (`.tShape()`); acceleration-based, not proportion-based

A constant-PWM `driveDist(30)` makes the chassis lurch — full power instantly at start, full stop instantly at end. That's fine for short moves but jarring for long ones, dangerous for fragile cargo (mosaic tiles, cement blocks), and accumulates yaw error during the abrupt acceleration window. The standard motion-control answer is a trapezoidal velocity profile: ramp from rest up to cruise, hold cruise, ramp back down to rest. Speed-vs-time traces out a trapezoid (sometimes called a "T-curve" in industry — the source of the local nickname "T-shape").

The implementation exposes the profile as a fluent `.tShape()` modifier on the existing Drive distance/time builders and on LineFollower's enable builder. Users opt in per call. Default endpoints are 0/0 (start at rest, end at rest); chained moves override via `.setStartSpeed(...)` / `.setEndSpeed(...)`. The shape is **acceleration-based**, not proportion-based: a single Config knob `TSHAPE_ACCEL_PWM_PER_CM` defines how many PWM units the speed gains per cm of encoder travel, and the per-side ramp distance falls out as `(cruise - endpoint) / accel`. This is more physically meaningful than "x% of the move is ramp-up" and makes chained moves natural — pass `endSpeed = cruise` and the formula returns 0, so no ramp-down at intermediate joins.

Per-side ramp distances are clamped to `[TSHAPE_RAMP_MIN_CM, TSHAPE_RAMP_MAX_CM]` so very long moves don't get a 50-cm ramp from a low accel and very short moves don't get a 1-mm ramp that defeats the smoothing. A further half-of-total cap on each side prevents the two ramps from overlapping. When acceleration plus deceleration distance exceeds the total move length, the profile collapses to a triangle: peak speed is reduced (`(start + end + cruise) / 3` heuristic, clamped to ≤ cruise) so the speed-vs-distance integral still matches the requested travel. A no-ramp epsilon (`TSHAPE_NO_RAMP_EPSILON_PWM = 2`) skips the ramp on the side where startSpeed or endSpeed is already at (or within 2 PWM of) cruise — needed for chained-move semantics where the floor clamp would otherwise force a tiny pointless ramp.

One small but useful convention: when the move is configured `.setBrake(false)`, the default endSpeed resolves to `cruise` instead of 0. The reasoning is that a no-brake move is presumably chaining into a follow-up move, so keeping speed up at the end matches caller intent. Explicit `.setEndSpeed(0)` overrides if a true coast-to-stop is wanted. This is the pattern of "smart defaults that infer intent from related setters" — same shape we should reuse on future builders where the relationship is similarly clear.

The profile runs as a cooperative state machine driven from `Drive.update()`. Each tick reads the encoder (or `millis()` for time-bounded moves), computes the traveled fraction, looks up the speed setpoint via the trapezoid formula, and issues the appropriate continuous primitive (`setSpeed`/`setSpeedSync`/`forwardGyro`) per the chosen `DriveMode`. The `_applyBias` discipline is preserved: RAW mode applies bias every tick, SYNC and GYRO modes do not. When `update()` sees a profile is active, it skips the lower-MCU "task done" drain so a stale completion bit from a prior bounded task can't falsely flip `_moveDone` mid-cruise (the W1-style guard from the fluent retrofit, applied here too).

For `driveTime`-with-tShape, the cm-based accel knob is reinterpreted as `rampMs ≈ rampCm / |cruise| * 1000` so the same Config constants apply. This is an approximation — there's no real distance for a time-bounded move — but it keeps the API uniform and behaves predictably enough that we haven't found a case where the discrepancy matters.

Blocking + tShape is supported but spins in the builder destructor with `delay(2)` until the profile completes. This is the second and only other allowed `delay()` site in the codebase after `calibrateMotorBias` (the iron rule against `delay()` is about non-blocking cooperative ticks, and a synchronous-orchestration call is by definition off the cooperative chain).

Turn ramping is **out of scope** for this implementation. `TurnGyro` is internally bounded by the lower MCU and intercepting it tick-by-tick to vary the power input would require writing our own continuous gyro-locked turn loop on top of raw `Move(L, -L)` plus IMU feedback. Possible but a separate project. Until then, turns happen at the configured `TURN_POWER` for their full duration. If smoothness matters during a turn, we re-evaluate after the chassis is on the track.

LineFollower's `.tShape()` is a simpler form: on enable, capture the start encoder, ramp the cruise PWM linearly from 0 to the resolved preset PWM over `(cruise / accel)` cm of travel (clamped to MIN/MAX). Once that distance is consumed, the ramp logic disables itself for the remainder of the enable session. There's no symmetric ramp-down on disable — disable is one-shot; if the planner wants a ramped stop, it can drop into a `Drive.driveDist(...).tShape().setStartSpeed(cruise)` to hand off explicitly.

## Drive command queue — sequential dispatch from the cooperative tick

The original RAII auto-execute pattern was correct for single moves but fundamentally broken for sequencing. When a caller writes:

```cpp
Drive.driveDist(30).tShape();
Drive.driveDist(20).tShape();
Drive.turnDeg(90);
```

all three builder destructors fire at their semicolons, BEFORE any `loop()` tick runs. Each subsequent destructor would cancel the in-flight work from the previous (the cancellation was even documented in `_execProfileMove`'s "silent — caller-discipline issue" comment). Net: only the *last* command actually runs. The future auto-path planner — which will issue dozens of moves in source order — would have been unusable.

The fix is a small FIFO command queue inside `DriveClass`. Builder destructors no longer dispatch directly: they capture the resolved parameters into a `DriveCommand` POD and push it onto the queue. `Drive.update()` runs three stages each tick: (1) drain any in-flight bounded MMLower task, (2) advance the trapezoidal-profile state machine, (3) if the chassis is fully idle and the queue is non-empty, pop and dispatch the next command. Stage ordering matters — profile completion in stage 2 must flip `_moveDone` before stage 3 looks at it, so a back-to-back chained queue dispatches the next command on the same tick the previous one finished, with no inter-segment gap.

Capacity is `Config::DRIVE_QUEUE_CAPACITY = 8` — comfortably more than any realistic multi-segment maneuver, and the per-slot `DriveCommand` struct is small enough that the whole array fits well under 1 KB of static RAM. Overflow drops the new command and prints a Serial warning; that's a caller bug worth surfacing rather than silently degrading. `Drive.stop()` clears the queue (true abort semantics — pending work is dropped) and `setBlocking(true)` bypasses the queue by spinning in `_drainBlocking()` until everything drains, which is the third allowed `delay()` site after `calibrateMotorBias` and the prior tShape-builder spins (the latter were removed by this change since `_drainBlocking` centralises the spin logic).

The queue model interacts subtly with the trapezoidal-profile DONE branch. The naive implementation called `stop(_profileBrake)` whenever a profile completed — but `stop()` itself calls `_clearQueue()` for abort semantics, which would silently wipe every queued follow-up command when a profiled move finished normally. The canonical example above (three default-brake-true profiles) would have lost moves 2 and 3 to that interaction. The fix splits the DONE branch by endSpeed: when `endSpeed == 0` we inline the equivalent of `stop(brake)` (`Move(0, 0)` plus `brake(_profileBrake)`) WITHOUT clearing the queue, and when `endSpeed != 0` we issue no motor command at all so the prior tick's setpoint (which is already at endSpeed) holds until the next queued command overwrites it. The "no-brake → endSpeed=cruise" convention from the tShape design becomes a natural way to express "let this move flow into the next" — both the chained semantics and the non-chained semantics now compose cleanly with the queue.

A second interaction is between LineFollower and queued Drive moves. LineFollower's `update()` writes `Drive.setSpeed(...)` every tick when enabled, which would race a queued bounded/profiled move on the motors. The queue dispatcher resolves this by auto-disabling LineFollower (with `brake=false`, since the new move is about to issue its own setpoints) the moment a new command pops off the queue. LineFollower stays disabled; the caller re-enables it explicitly when they want the follower running again. This makes Drive bounded/profiled moves "own" the motors exclusively for their duration without any caller-side discipline.

A defensive guard worth recording: each `_execXxx` async path only sets `_moveDone = false` when the lower MCU returned `0x00`. Without this, a failed dispatch (e.g. `MoveDegs` rejecting because the lower MCU is in a transient bad state) would leave `_moveDone == false` forever, hanging `_drainBlocking()` on a task that never started. The guard mirrors the C1/W1 pattern already in the turn execs.

The blocking variant deserves its own paragraph. `setBlocking(true)` bypasses the queue's async semantics by *draining* it: the destructor calls `_drainBlocking()` (which spins until queue is empty AND chassis is idle), then enqueues the new command, then drains again. The result is a synchronous-looking call on top of the queue infrastructure. Reserve blocking for command-time orchestration (calibration routines, mission-step transitions where the next decision genuinely needs the move to be complete) — never use it inside any module's `update()` because the spin would block the cooperative tick chain.
