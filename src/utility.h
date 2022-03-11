#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <data_buff.h>
#include <Comms.h>
#include <config.h>

#define USE_DASHBOARD 1

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   avoid using pins with LEDs attached

namespace utility {
    // String inString;
    void runMotor(double speed);
    double encoderToAngle(double encoderValue);
    double voltageToPressure(double voltage);
    double voltageToHighPressure(double voltage);
    double readPot();
    int waitConfirmation();
}