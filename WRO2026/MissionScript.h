/**
 * @file MissionScript.h
 * @brief Declaration of the hand-typed competition mission.
 * @author WRO 2026
 *
 * @details
 * `buildMission()` is the single authored mission script. It is defined in
 * `MissionScript.cpp` so the same translation unit can be compiled into both
 * the firmware sketch and a host build (it depends only on the `Mission`
 * fluent DSL — see `Mission.h`). Edit the body in `MissionScript.cpp` to
 * change the run.
 */

#ifndef MISSIONSCRIPT_H
#define MISSIONSCRIPT_H

/**
 * @brief Hand-typed PR3 mission. Edit this function to change tomorrow's run.
 *
 * Every `Mission.<verb>(...)` enqueues a step. `Mission.start()` arms the
 * queue and returns immediately; `loop()` drains it via `Mission.update()`.
 */
void buildMission();

#endif // MISSIONSCRIPT_H
