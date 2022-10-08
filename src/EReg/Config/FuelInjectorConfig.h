//Fuel Injector EReg Config 
#pragma once
#include "../FlowProfiles.h"

namespace Config {

    #define MAX_ANGLE 900                                                    
    #define MIN_ANGLE 0
    #define ANTIWINDUP_RANGE_LOWER 200
    #define ANTIWINDUP_RANGE_UPPER 900

    // Controller Constants
    const double p_outer_nominal = 1.0, i_outer_nominal = 0.7e-6, d_outer_nominal = 0.06; // nominal is 4000 -> 500 psi flow
    const double p_inner = 9, i_inner = 3.5e-6, d_inner = 0.15;

    // Flow Parameters
    const float pressureSetpoint = 460;
    const unsigned long loxLead = 0; //time in milliseconds

    // Diagnostic configs
    const int servoTestPoints = 9;
    const float servoTravelInterval = 100; // encoder counts
    const unsigned long servoSettleTime =  1000UL * 1000UL; // micros
    const float stopDiagnosticPressureThresh = 700; // diagnostic terminates if either tank exceeds this
    const float diagnosticSpeed = 0;

    // const float LOW_PT_C = 13.482;
    // const float LOW_PT_M = 1.2407;

    // const float HIGH_PT_C = 5.0;
    // const float HIGH_PT_M = 0.7973;
    // // true = (measured - LOW_PT_C) / LOW_PT_M
}