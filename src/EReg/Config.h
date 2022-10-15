#if defined(FUEL)
    // 0.804 for ln2, 0.493 for propane
    #define PROPELLANT_GRAVITY 0.804
    #if defined(IS_INJECTOR)
        #include "Config/FuelInjectorConfig.h"
    #else
        #include "Config/FuelTankConfig.h"
    #endif
#elif defined(LOX)
    // 0.804 for ln2, 1.14 for lox
    #define PROPELLANT_GRAVITY 0.804
    #if defined(IS_INJECTOR)
        #include "Config/LoxInjectorConfig.h"
    #else
        #include "Config/LoxTankConfig.h"
    #endif
#endif

#pragma once

namespace Config {

    #define ESP_ADDRESS_1 25
    #define ESP_ADDRESS_2 26
    #define ESP_ADDRESS_3 27
    #define ESP_ADDRESS_4 28

    #define MAX_SPD 255
    #define MIN_SPD -255

    #define OPEN_LOOP_SPEED 200                                                                                                                                                  

    #define INNER_BUFFER_SIZE 2
    #define DIAGNOSTIC_BUFFER_SIZE 5

    const unsigned long telemetryInterval = 50 * 1000UL; // time in microseconds between telemetry packets
    const unsigned long telemetryIntervalIdle = 100 * 1000UL; // time in microseconds between telemetry packets

    const unsigned long closeTime = 3UL * 1000UL; // time in milliseconds

    // flow duration
    const unsigned long flowDuration = 18UL * 1000UL * 1000UL; // time in microseconds TODO change to 5s
    const unsigned long rampDuration = 500UL * 1000UL; // time in microseconds

    // Pressurization Parameters
    const unsigned long pressurizationRampDuration = 30 * 1000UL * 1000UL;
    const float pressurizationCutoff = pressureSetpoint * 0.99;
    const float pressurizationStartPressure = 100;

    // Diagnostic configs
    const float minAngleMovement = 300;
    const float servoSettleThresh = 10; // encoder counts
    const float initialServoAngle = 100; // encoder counts

    // Abort Thresholds
    const float abortPressureThresh = 750; // transition to idleClosed if prop tank exceeds this

    // Injector Feedforward Thresholds
    const float minInjectorFeedforwardAngle = 200;
    const float maxInjectorFeedforwardAngle = 900;

    // Number of last readings that should be recorded
    const int numReadingsStored = 3;
}