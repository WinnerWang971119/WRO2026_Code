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
        //.task("emptyTest");
        //.task("dumpMosaic")
        //.task("pot")
        //.task("rectangleShovel");
        .task("triangleShovel")
        .task("pot");
        //.task("rectangleShovel");
        //.task("debries");*/
        //.task("testOne");
        //.task("pot");
        //.task("triangleShovel");
    Mission.start();
}
