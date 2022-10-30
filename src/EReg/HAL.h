#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <ADS1X15.h>
#include <ESP32Encoder.h>
#include "Ducers.h"

namespace HAL {
    // main valve pins
    const int mainValve1 = 33;
    const int mainValve2 = 17;
    
    // ereg motor pins
    const int motor1 = 4;
    const int motor2 = 2;
    const int motor3 = 33;
    const int motor4 = 17;

    const int motor1Channel = 0;
    const int motor2Channel = 1;
    const int motor3Channel = 2;
    const int motor4Channel = 3;


    // Best encoder Performance: both pins have interrupt capability
    // avoid using pins with LEDs attached
    const int enc1 = 36;
    const int enc2 = 39;

    extern ADS1115 adc;
    extern ESP32Encoder encoder;

    #ifndef IS_INJECTOR
    const uint8_t upstreamPT = 2; // pressurant
    const uint8_t downstreamPT = 1; // tank
    #else
    const uint8_t upstreamPT = 1; // tank
    const uint8_t downstreamPT = 0; // injector
    #endif
    

    const int daqEndIp = 29;
    const int acEndIp = 21;
    // const int fuelTankEndIp = 25;
    // const int loxFuelEndIp = 26;
    // const int fuelInjectorEndIp = 27;
    // const int loxInjectorEndIp = 28;

    const int eregAbortID = 201;

    void init();
    float readUpstreamPT();
    float readDownstreamPT();
}
