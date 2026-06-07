/**
 * @file MissionScript.cpp
 * @brief Definition of the hand-typed competition mission.
 * @author WRO 2026
 */

#include "MissionScript.h"

#include "Mission.h"

void buildMission()
{
    Mission
        .print("MISSION GO")
        .led(0, 0, 255, 200)        // brief blue flash on arm
        .goTo("B")
        .beep(1000, 150)
        .print("AT B")
        .wait(500)
        .goTo("G")                  // blind hop from B to G
        .beep(1500, 300)
        .led(0, 255, 0, 500)
        .print("AT G")
        .wait(800)
        .goTo("B")                  // back to the line
        .goTo("C")
        .beep(800, 150)
        .print("AT C")
        .wait(500)
        .goTo("D")
        .print("AT D")
        .wait(500)
        .goTo("A")
        .led(255, 255, 255, 1000)
        .task("demoEcho", uint8_t(7), uint8_t(42))
        .print("DONE");
    Mission.start();
}
