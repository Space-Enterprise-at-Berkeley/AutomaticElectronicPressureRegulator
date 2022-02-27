#pragma once

#include <controls.h>
#include <Comms.h>

namespace tests {

    void motorDirTest();
    void motorPowerTest();
    void ptTest();
    void potTest();
    void servoTest();
    void angleSweep(long startAngle, long endAngle, unsigned long flowDuration, long extraTime);
    boolean pressureize_tank();
};