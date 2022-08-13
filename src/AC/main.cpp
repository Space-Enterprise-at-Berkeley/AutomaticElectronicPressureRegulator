#include <Common.h>
#include "Comms.h"
#include "Actuators.h"
#include "Power.h"
#include "Ereg.h"
#include "HAL.h"
#include "Toggles.h"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

Task taskTable[] = {
    // actuators
    // {Actuators::propTankVentRBVSample, 0},
    // {Actuators::loxTankVentRBVSample, 0},
    // {Actuators::propFillRBVSample, 0},
    // {Actuators::loxFillRBVSample, 0},
    // {Actuators::twoWaySample, 0},
    // {Actuators::loxGemsSample, 0},
    // {Actuators::igniterSample, 0},

    {Actuators::stopPropFillRBV, 0, false},
    {Actuators::stopLoxTankVentRBV, 0, false},
    {Actuators::stopPropFillRBV, 0, false},
    {Actuators::stopLoxFillRBV, 0, false},
    {Actuators::stopTwoWay, 0, false},
    {Actuators::stopLoxGems, 0, false},
    {Actuators::stopIgniter, 0, false},

    // // power
    // {Power::battSample, 0},
    // {Power::supply12Sample, 0},

    // ereg
    {EReg::sampleTelemetry, 0},

    // toggles
    // {Toggles::fuelGemsSample, 0},
    // {Toggles::breakWireSample, 0},
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

int main() {
    // hardware setup
    Serial.begin(115200);
    #ifdef DEBUG_MODE
    while(!Serial) {} // wait for user to open serial port (debugging only)
    #endif
    Actuators::stopPropTankVentRBVTask = &taskTable[4];
    Actuators::stopLoxTankVentRBVTask = &taskTable[5];
    Actuators::stopPropFillRBVTask = &taskTable[6];
    Actuators::stopLoxFillRBVTask = &taskTable[7];
    Actuators::stopTwoWayTask = &taskTable[8];
    Actuators::stopLoxGemsTask = &taskTable[9];
    Actuators::stopIgniterTask = &taskTable[10];

    HAL::initHAL();
    Comms::initComms();
    Actuators::initActuators();
    // Power::initPower();
    EReg::initEReg();
    Toggles::initToggles();

    while(1) {
        uint32_t ticks = micros(); // current time in microseconds
        for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
        if (taskTable[i].enabled && ticks >= taskTable[i].nexttime) {
            // DEBUG(i);
            // DEBUG("\n");
            taskTable[i].nexttime = ticks + taskTable[i].taskCall();
        }
        }
        Comms::processWaitingPackets();
    }
    return 0;
}
