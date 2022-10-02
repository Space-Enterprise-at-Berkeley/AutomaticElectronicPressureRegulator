#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <ESP32Encoder.h>

namespace HAL {
    // main valve pins
    const int mainValve1 = 33;
    const int mainValve2 = 17;
    
    // ereg motor pins
    const int motor1 = 2;
    const int motor2 = 4;

    const int motor1Channel = 0;
    const int motor2Channel = 1;


    // Best encoder Performance: both pins have interrupt capability
    // avoid using pins with LEDs attached
    const int enc1 = 39;
    const int enc2 = 36;

    extern Adafruit_ADS1115 adc;
    extern ESP32Encoder encoder;

    const uint8_t hpPT = 2;
    const uint8_t lpPT = 1;

    const int generalStartFlowID = 52;
    const int eregStartFlowID = 52;
    const int generalAbortID = 51;
    const int eregAbortID = 1;
    const int daqEndIp = 22;
    const int acEndIp = 21;
    const int fuelInjectorEndIp = 31;
    const int loxInjectorEndIp = 32;
    const int fuelTankEndIp = 33;
    const int loxFuelEndIp = 34;


    void init();
}
