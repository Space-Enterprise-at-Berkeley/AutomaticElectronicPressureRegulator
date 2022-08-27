#if defined(LOX)
    #include "LOX_Config.h"
#elif defined(FUEL)
    #include "Fuel_Config.h"
#endif

#pragma once

namespace Config {

    #ifdef EVERY
    #define SERIAL_COMMS Serial1
    #else
    #define SERIAL_COMMS Serial
    #endif

    #define MAX_SPD 255
    #define MIN_SPD -255
    #define MAX_ANGLE 500                                                    
    #define MIN_ANGLE 0
    #define ANTIWINDUP_RANGE_LOWER 150
    #define ANTIWINDUP_RANGE_UPPER 400

    #define OPEN_LOOP_SPEED 200                                                                                                                                                  

    #define OUTER_BUFFER_SIZE 5
    #define INNER_BUFFER_SIZE 2
    #define DIAGNOSTIC_BUFFER_SIZE 5

    const unsigned long telemetryInterval = 5000UL; // time in microseconds between telemetry packets

    const unsigned long closeTime = 3UL * 1000UL; // time in milliseconds

    // flow duration
    const unsigned long flowDuration = 30UL * 1000UL * 1000UL; // time in microseconds TODO change to 5s
    const unsigned long rampDuration = 500UL * 1000UL; // time in microseconds

    // Pressurization Parameters
    const unsigned long pressurizationRampDuration = 30 * 1000UL * 1000UL;
    const float pressurizationCutoff = pressureSetpoint * 0.99;
    const float pressurizationStartPressure = 100;

    // Diagnostic configs
    const float minAngleMovement = 300;
    const int servoTestPoints = 5;
    const float servoTravelInterval = 100; // encoder counts
    const unsigned long servoSettleTime = 200UL * 1000UL; // micros
    const float servoSettleThresh = 10; // encoder counts

    // Abort Thresholds
    const float abortPressureThresh = 750; // transition to idleClosed if prop tank exceeds this
    const float stopDiagnosticPressureThresh = 100; // diagnostic terminates if either tank exceeds this

}