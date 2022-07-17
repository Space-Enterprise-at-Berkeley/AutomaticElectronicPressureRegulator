#pragma once

#include <stdint.h>

namespace HAL {
    #define MOTOR1 10
    #define MOTOR2 9
    #define MOTOR3 6
    #define MOTOR4 5
    #define ENC1 3
    #define ENC2 2
    #define POTPIN A1
#define HP_PT A4
#define LP_PT A0
#define INJECTOR_PT A3

    const int motor1 = 10;
    const int motor2 = 9;
    const int motor3 = 6;
    const int motor4 = 5;

    const int enc1 = 3;
    const int enc2 = 2;

    const uint8_t hpPT = A4;
    const uint8_t lpPT = A0;
    const uint8_t injectorPT = A3;

    const uint8_t potPin = A1;
}