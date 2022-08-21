#pragma once

#include <stdint.h>

namespace HAL {
    // main valve pins
    const int mainValve1 = 10;
    const int mainValve2 = 9;
    
    // ereg motor pins
    const int motor1 = 6;
    const int motor2 = 5;

    const int enc1 = 2;
    const int enc2 = 3;

    const uint8_t hpPT = A1;
    const uint8_t lpPT = A2;
    const uint8_t injectorPT = A3;


    const uint8_t potPin = A1;
}