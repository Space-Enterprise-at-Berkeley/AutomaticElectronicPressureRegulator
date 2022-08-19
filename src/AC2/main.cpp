#include "Common.h"
#include "Comms.h"
#include "Actuators.h"
#include "HAL.h"
#include "ERegBoard.h" //TODO removing this breaks compilation.. why?

#include <Arduino.h>
#include <Wire.h>
#define DEBUG_MUL
#include <SPI.h>

Task taskTable[] = {
    // actuators
    {Actuators::fuelFillRBVSample, 0},
    {Actuators::loxFillRBVSample, 0},
    {Actuators::pressFillRBVSample, 0},
    {Actuators::pressLineVentRBVSample, 0},
    // {Actuators::act5Sample, 0},
    // {Actuators::act6Sample, 0},
    // {Actuators::act7Sample, 0},

    {Actuators::stopFuelFillRBV, 0, false},
    {Actuators::stopLoxFillRBV, 0, false},
    {Actuators::stopPressFillRBV, 0, false},
    {Actuators::stopPressLineVentRBV, 0, false},
    // {Actuators::stopAct5, 0, false},
    // {Actuators::stopAct6, 0, false},
    // {Actuators::stopAct7, 0, false},
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

int main() {
    // hardware setup
    Serial.begin(115200);
    #ifdef DEBUG_MODE
    while(!Serial) {} // wait for user to open serial port (debugging only)
    #endif
    Actuators::stopFuelFillRBVTask = &taskTable[7];
    Actuators::stopLoxFillRBVTask = &taskTable[8];
    Actuators::stopPressFillRBVTask = &taskTable[9];
    Actuators::stopPressLineVentRBVTask = &taskTable[10];
    // Actuators::stop5 = &taskTable[11];
    // Actuators::stop6 = &taskTable[12];
    // Actuators::stop7 = &taskTable[13];

    DEBUGLN("Starting AC2");

    HAL::initHAL();
    Comms::initComms();
    Actuators::initActuators();

    while(1) {
        uint32_t ticks = micros(); // current time in microseconds
        for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
        if (taskTable[i].enabled && taskTable[i].nexttime - ticks > UINT32_MAX / 2) {
            taskTable[i].nexttime = ticks + taskTable[i].taskCall();
        }
        }
        Comms::processWaitingPackets();
    }
    return 0;
}
