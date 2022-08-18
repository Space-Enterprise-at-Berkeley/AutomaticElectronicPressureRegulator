#pragma once

#include <stdint.h>

namespace HAL {
    // main valve pins
    const int mainValve1 = 10;
    const int mainValve2 = 9;
    
    // ereg motor pins
    const int motor1 = 6;
    const int motor2 = 5;

    const int enc1 = 3;
    const int enc2 = 2;

    const uint8_t hpPT = A5;
    const uint8_t lpPT = A0;
    const uint8_t injectorPT = A3;


    const uint8_t potPin = A1;
}