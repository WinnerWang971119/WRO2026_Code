# PR Code Review

Review target: local branch `phase1/drive-line-intersection` against `origin/main`,
including the current workspace changes.

Scope reviewed:

- `WRO2026/Drive.h`
- `WRO2026/Drive.cpp`
- `WRO2026/LineFollower.h`
- `WRO2026/LineFollower.cpp`
- `WRO2026/Intersection.cpp`
- `WRO2026/EventTrace.cpp`
- `WRO2026/WRO2026.ino`

## Summary

The biggest risks are not style issues. They are robot-control failure modes:
reverse motion can initially move in the wrong direction, synchronous drive calls can
block forever, and failed queued commands can be treated as completed. Those are
dangerous because the future mission planner will assume "the command finished" means
"the robot is now physically where the command was supposed to place it."

The second group of issues affects reliability and debuggability: telemetry can consume
intersection events, intersection detection has no debounce, and setup can report "ready"
even after hardware initialization failed.

## Finding 1 - P1 - Reverse profiled moves start in the wrong direction

Location: `WRO2026/Drive.h:1126`

Problem:

With `tShape` now default-on, a reverse move such as:

```cpp
Drive.driveDist(30).setPower(-50);
```

resolves `cruise = -50`, but the default `startSpeed` is
`Config::TSHAPE_START_SPEED`, which is positive. That means the trapezoid starts at a
positive PWM and ramps toward a negative cruise speed.

Why this is an error:

The public API says reverse travel is expressed by negative power, not negative
distance. If a caller asks for negative power, every emitted setpoint for that segment
must be in the reverse direction unless the caller explicitly requests otherwise. A
positive start setpoint violates the command contract and can make the chassis move
forward before reversing.

Robot-control impact:

This can miss a line, bump a game element, or ruin a close-position maneuver because the
first few ticks of a reverse move go the wrong way.

Improvement direction:

Resolve the default start speed with the sign of the cruise speed. For example, a
negative cruise should default to `-Config::TSHAPE_START_SPEED`. Apply the same rule to
`DriveTimeBuilder`.

## Finding 2 - P1 - Reverse line-follow ramp targets forward PWM

Location: `WRO2026/LineFollower.cpp:287`

Problem:

When reverse line following is enabled, `update()` negates the cruise speed:

```cpp
if (_reverse)
{
    cruise = (int16_t) -cruise;
}
```

But the enable-time ramp target is seeded as:

```cpp
_tShapeTargetPwm = presetToPwm(_preset);
```

That target is always positive.

Why this is an error:

The reverse flag is part of the enable transition contract. If the controller is enabled
in reverse, both the steady cruise and the ramp target must use the reverse sign.
Currently the tShape ramp can override the correctly-negated cruise with a positive
interpolated value.

Robot-control impact:

Reverse line following can initially drive forward, exactly when the robot is likely in
a constrained position near an intersection or turn setup.

Improvement direction:

Compute the resolved target PWM after applying the reverse flag, then store that value
in `_tShapeTargetPwm`.

## Finding 3 - P1 - Blocking drain can hang forever

Location: `WRO2026/Drive.cpp:1014`

Problem:

`_drainBlocking()` loops while the chassis or queue is not idle:

```cpp
while (!_isIdle() || _qCount > 0)
```

There is no timeout, no maximum iteration count, and no emergency abort path.

Why this is an error:

The drive builders are sync-blocking by default, so this path is not rare. If the lower
MCU never reports task completion, an encoder stops changing, a wheel stalls, or a
profile target is never reached, the robot can remain trapped in this loop.

Robot-control impact:

The mission scheduler and the 2-minute time guard cannot recover if code is stuck in
this blocking drain. In a competition run, that can turn one failed movement into a
full-run failure.

Improvement direction:

Add a bounded timeout based on the command type and expected duration/distance. On
timeout, stop the drive, clear the queue, mark an error state, and surface the failure
through Serial/OLED so the mission layer can recover.

## Finding 4 - P1 - Invalid negative bounds can become huge moves

Location: `WRO2026/Drive.h:1119`

Problem:

The API documents distance as non-negative, with reverse direction expressed through
negative power. However, negative distances can still be enqueued. If the move is below
the tShape threshold, it falls back to a non-profiled command and can reach
`cmToEncDeg()`, which casts the computed float to `uint16_t`.

Why this is an error:

An invalid negative distance should be rejected or clamped before dispatch. Allowing it
to reach a `uint16_t` conversion can turn a caller mistake into a very large encoder
target.

Robot-control impact:

A typo such as `Drive.driveDist(-3)` could become a long unexpected move instead of a
small reverse correction or a harmless rejected command.

Improvement direction:

Validate the bounds before enqueueing commands and again before dispatching executor
paths. Reject negative `cm` and `sec` consistently for both profiled and non-profiled
moves.

## Finding 5 - P2 - Failed queued commands are silently skipped

Location: `WRO2026/Drive.cpp:912`

Problem:

`_dispatchNext()` pops the command from the queue before dispatching it. The executor
return code is ignored:

```cpp
_execDriveDistSync(cmd.power, cmd.cm, cmd.brake, false);
```

If dispatch fails, the command has already been removed and the queue continues.

Why this is an error:

The queue is a sequencing contract. A failed segment must stop the sequence or enter an
error state. Dropping the command and continuing makes the software believe motion
happened when the lower MCU rejected it.

Robot-control impact:

The robot's physical pose can diverge from the mission planner's assumed pose. Every
later turn, line-follow enable, or manipulator action can be based on a position the
robot never reached.

Improvement direction:

Capture executor return codes. If dispatch fails, stop the motors, clear or freeze the
queue, record the failing command/error code, and expose a query such as
`Drive.hasFault()` for higher-level recovery.

## Finding 6 - P2 - Telemetry consumes control intersection events

Location: `WRO2026/EventTrace.cpp:82`

Problem:

`EventTrace.update()` calls:

```cpp
Intersection.isAtIntersection()
```

That method is read-and-clear.

Why this is an error:

Telemetry should observe control events, not consume them. Once path tracking or
localization is added, it will also need to read intersection events. If EventTrace runs
first or is the only code that sees the one-shot flag, the navigation layer can miss the
event.

Robot-control impact:

The robot can physically cross an intersection while the logger displays it correctly,
but the future navigation stack fails to advance its node count.

Improvement direction:

Add a non-consuming `peek` API, split "consume" and "last event" accessors, or have
Intersection publish copied events to EventTrace instead of letting EventTrace clear the
control signal.

## Finding 7 - P2 - Intersection detection needs debounce and raw validation

Location: `WRO2026/Intersection.cpp:43`

Problem:

The detector treats any non-`NONE`, non-`UNKNOWN` raw value as a valid junction and emits
a rising edge whenever `_wasAtJunction` was false. A single noisy `NONE` or `UNKNOWN`
sample while still physically over the same junction resets `_wasAtJunction`.

Why this is an error:

Physical line sensors can flicker at junction edges, under lighting changes, or during
fast motion. Edge detection without a stable-release debounce can turn one physical
intersection into multiple software intersections. Unknown raw values should also be
rejected instead of cast into the enum.

Robot-control impact:

Duplicate intersection events can make localization advance too many nodes. That is one
of the worst navigation failures because the robot keeps moving confidently with a wrong
map position.

Improvement direction:

Require a valid junction raw value for a small number of consecutive ticks before
firing, and require a stable `NONE` period before arming the next edge. Reject raw values
outside the known enum range.

## Finding 8 - P2 - Boot reports ready after init failures

Location: `WRO2026/WRO2026.ino:102`

Problem:

`setup()` calls module `begin()` methods but ignores their return values, then prints:

```cpp
Serial.println("WRO2026 Phase 1 ready.");
```

Why this is an error:

Several `begin()` methods return `bool` specifically to report setup success or failure.
Ignoring those values makes a failed drivebase or sensor initialization look identical
to a clean boot.

Robot-control impact:

The team can start a run with a failed tracer bind, failed drive setup, or failed
display initialization and only discover it after the robot moves incorrectly.

Improvement direction:

Accumulate setup status. If any required module fails, show a clear Serial/OLED error
and prevent normal motion until the problem is fixed or a deliberate override is used.

## Finding 9 - P2 - MatrixLineTracer include path may not resolve

Location: `WRO2026/LineFollower.h:44`

Problem:

The code includes:

```cpp
#include <MiniR4_MXLineTracer.h>
```

The referenced MatrixMiniR4 library stores that header under:

```text
src/Modules/Sensors/MiniR4_MXLineTracer.h
```

Why this is an error:

Arduino library include paths are not guaranteed to include every nested source
directory as a direct header root. A build that only exposes the library `src` root may
fail to find `MiniR4_MXLineTracer.h` directly.

Robot-control impact:

This is a build reliability issue rather than a runtime control issue. If it fails, the
sketch will not compile on another machine or Arduino IDE configuration.

Improvement direction:

Use an include path that matches the vendor library layout, or include the tracer type
through the vendor facade if that is the intended public API.

## Verification Notes

- `git diff --check` reported no whitespace errors during review.
- `arduino-cli` is not installed in this workspace, so a headless UNO R4 compile was not run.
- The MatrixMiniR4 source was inspected locally under
  `C:\Users\wangd\documents\WRO2026\MatrixMiniR4\` to confirm the drive and line tracer API
  shapes used by the findings.
