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
        .task("testChassis", 10);
    Mission.start();
}
