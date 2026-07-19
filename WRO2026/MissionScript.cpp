/**
 * @file MissionScript.cpp
 * @brief Definition of the hand-typed competition mission.
 * @author WRO 2026
 */

#include "MissionScript.h"

#include "src/mission/Mission.h"

void buildMission()
{
    Mission
        .task("readMosaic").task("rectangleShovel").task("pot").task("triangleShovel");
    Mission.start();
}
