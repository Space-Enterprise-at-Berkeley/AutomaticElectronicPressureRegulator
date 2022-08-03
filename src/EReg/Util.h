#pragma once

#include <PIDController.h>

namespace Util {

    #define MAX_SPD 255
    #define MIN_SPD -255
    // #define STATIC_SPD 60
    #define STATIC_SPD 0
    //old max angle = 1296
    //3200*48/26
    #define MAX_ANGLE 363
    #define MIN_ANGLE 0

    extern double p_outer, i_outer, d_outer;
    extern double p_inner, i_inner, d_inner;

    double encoderToAngle(double encoderValue);
    double voltageToLowPressure(double voltage);
    double voltageToHighPressure(double voltage);
    double readPot();
    double compute_feedforward(double pressure_setpoint, double hp);
    void runMotors(float speed);
    PIDController* getInnerController();
    PIDController* getOuterController();
}