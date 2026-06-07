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
        .task("testOne")
        .goTo("final")
        .wait(500)
        .goTo("Inter4");
    Mission.start();
}
