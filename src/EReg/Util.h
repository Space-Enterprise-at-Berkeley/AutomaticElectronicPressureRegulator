#pragma once

#include <PIDController.h>
#include <Encoder.h>

namespace Util {

    struct PidConstants {
        double k_p;
        double k_i;
        double k_d;
    };

    double encoderToAngle(double encoderValue);
    double voltageToLowPressure(double voltage);
    double voltageToHighPressure(double voltage);
    double compute_feedforward(double pressureSetpoint, double hp);
    PidConstants computeDynamicPidConstants(double highPressure, double lowPressure);
    double clip(double value, double minOutput, double maxOutput);
    void runMotors(float speed);
    
    PIDController* getInnerController();
    PIDController* getOuterController();
    Encoder* getEncoder();
}