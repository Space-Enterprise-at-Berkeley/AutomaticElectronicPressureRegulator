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
    const int motor1 = 4;
    const int motor2 = 2;

    // Best encoder Performance: both pins have interrupt capability
    // avoid using pins with LEDs attached
    const int enc1 = 36;
    const int enc2 = 39;

    extern Adafruit_ADS1115 adc;
    extern ESP32Encoder encoder;

    const uint8_t hpPT = 2;
    const uint8_t lpPT = 1;

    void init();
}
