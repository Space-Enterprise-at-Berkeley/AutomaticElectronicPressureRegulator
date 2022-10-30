#pragma once

#include <PIDController.h>
#include <Arduino.h>
#include "HAL.h"
#include "Config.h"

namespace Util {

    #ifndef min
    #define min(a,b) (((a) < (b)) ? (a) : (b))
    #endif

    #ifndef max
    #define max(a,b) (((a) > (b)) ? (a) : (b))
    #endif

    struct PidConstants {
        double k_p;
        double k_i;
        double k_d;
    };

    double encoderToAngle(double encoderValue);
    double voltageToLowPressure(double voltage);
    double voltageToHighPressure(double voltage);
    double compute_feedforward(double pressureSetpoint, double hp, unsigned long flowTime);
    double compute_injector_feedforward(double pressureSetpoint, double tankPressure, double flowRate);
    double injector_characterization(unsigned long flowTime);
    PidConstants computeTankDynamicPidConstants(double highPressure, double lowPressure, unsigned long flowTime);
    PidConstants computeInjectorDynamicPidConstants(unsigned long flowTime);
    double heartBeat(unsigned long time);
    double clip(double value, double minOutput, double maxOutput);
    void runMotors(float speed);
    void runInjectorMotors(float speed);
    
    PIDController* getInnerController();
    PIDController* getOuterController();
}