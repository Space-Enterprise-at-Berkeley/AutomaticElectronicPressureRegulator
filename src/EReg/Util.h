#pragma once

#include <PIDController.h>
#include <Encoder.h>

namespace Util {
    double encoderToAngle(double encoderValue);
    double voltageToLowPressure(double voltage);
    double voltageToHighPressure(double voltage);
    double compute_feedforward(double pressure_setpoint, double hp);
    void runMotors(float speed);
    double clip(double value, double minOutput, double maxOutput);
    PIDController* getInnerController();
    PIDController* getOuterController();
    Encoder* getEncoder();
}