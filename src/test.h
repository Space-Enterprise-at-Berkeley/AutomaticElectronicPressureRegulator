#pragma once

#include <controls.h>
#include <Comms.h>
#include <utility.h>

namespace tests {
    
    // long angle;
    // unsigned long lastPrint;
    // int setPoint = 130;
    // double motorAngle;
    // double potAngle;
    // double HPpsi;
    // double LPpsi;
    // double InjectorPT;
    bool isPrint = true;
    unsigned int printFreq;
    // String inString="";

    //Servo Characterization
    unsigned long flowStart = millis(); // in millis
    // unsigned long flowDuration;

    void motorDirTest();
    void motorPowerTest();
    void ptTest();
    void potTest();
    void servoTest();
    void angleSweep(long startAngle, long endAngle, unsigned long flowDuration, long extraTime);
    boolean pressureize_tank();
};