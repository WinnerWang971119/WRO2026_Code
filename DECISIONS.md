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
