#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <data_buff.h>
#include <Comms.h>

#define MOTOR1 6
#define MOTOR2 5
#define ENC1 2
#define ENC2 3

#define PRESSURE_MAXIMUM 1000
// #define STATIC_SPD 60
// #define STATIC_SPD 0

#define POTPIN A0
#define HP_PT A1
#define LP_PT A2
#define INJECTOR_PT A3

#define BUFF_SIZE 5

#define USE_DASHBOARD 1

int speed;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   avoid using pins with LEDs attached

Encoder encoder(ENC1, ENC2);

namespace utility {
    void runMotor();
    double encoderToAngle(double encoderValue);
    double voltageToPressure(double voltage);
    double voltageToHighPressure(double voltage);
    double readPot();
    int waitConfirmation();
}