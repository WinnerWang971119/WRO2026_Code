#include "Manipulator.h"

// Global singleton instance declared in Manipulator.h.
ManipulatorClass Manipulator;

bool ManipulatorClass::begin()
{
    return true;
}

void ManipulatorClass::update()
{
    /* no-op */
}

void ManipulatorClass::setLift(uint8_t /*height*/)
{
    /* no-op */
}

void ManipulatorClass::setGripper(uint8_t /*angle*/)
{
    /* no-op */
}

void ManipulatorClass::setArm(uint8_t /*angle*/)
{
    /* no-op */
}

bool ManipulatorClass::isDone() const
{
    return true;
}
