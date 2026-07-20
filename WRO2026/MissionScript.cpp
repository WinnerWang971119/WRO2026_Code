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
        .task("readMosaic").task("triangleShovel").task("rectangleShovel").task("pot").task("debries");
    Mission.start();
}
