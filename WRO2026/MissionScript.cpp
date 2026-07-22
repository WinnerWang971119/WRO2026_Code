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
        //.task("readMosaic")
        //.task("mosaic");
        /*.task("dumpMosaic")
        .task("pot")
        .task("rectangleShovel")
        .task("triangleShovel")
        .task("debries");*/
        .task("testOne");
        //.task("pot")
        //.task("triangleShovel");
    Mission.start();
}
