#pragma once

#include <PIDController.h>
#include <Encoder.h>

namespace Util {
    enum ValveAction { MAIN_VALVE_OPEN, MAIN_VALVE_CLOSE };
    double encoderToAngle(double encoderValue);
    double voltageToLowPressure(double voltage);
    double voltageToHighPressure(double voltage);
    double compute_feedforward(double pressure_setpoint, double hp);
    void runMotors(float speed);
    void actuateMainValve(ValveAction action);
    PIDController* getInnerController();
    PIDController* getOuterController();
    Encoder* getEncoder();
}