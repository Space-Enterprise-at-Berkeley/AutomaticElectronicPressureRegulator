#include <Common.h>
#include "../Comms.h"
#include "Actuators.h"
#include "Ereg.h"
#include "HAL.h"
#include "Toggles.h"
#include "Automation.h"

#include <Arduino.h>
#include <Wire.h>
#define DEBUG_MUL
#include <SPI.h>

Task taskTable[] = {
    // actuators
    {Actuators::fuelTankVentRBVSample, 0},
    {Actuators::loxTankVentRBVSample, 0},
    {Actuators::igniterEnableSample, 0},
    {Actuators::twoWaySample, 0},
    {Actuators::fuelGemsSample, 0},
    {Actuators::loxGemsSample, 0},

    {Actuators::stopFuelTankVentRBV, 0, false},
    {Actuators::stopLoxTankVentRBV, 0, false},
    {Actuators::stopIgniterRelay, 0, false},
    {Actuators::stopTwoWay, 0, false},
    {Actuators::stopFuelGems, 0, false},
    {Actuators::stopLoxGems, 0, false},

    //Automation
    {Automation::autoventFuelGemValveTask},
    {Automation::autoventLoxGemValveTask},

    // ereg
    {EReg::sampleFuelEregTelemetry, 0},
    {EReg::sampleLoxEregTelemetry, 0},

    // toggles
    {Toggles::igniterSample, 0},
    {Toggles::breakWireSample, 0},
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

int main() {
    // hardware setup
    Serial.begin(115200);
    #ifdef DEBUG_MODE
    while(!Serial) {} // wait for user to open serial port (debugging only)
    #endif
    Actuators::stopFuelTankVentRBVTask = &taskTable[7];
    Actuators::stopLoxTankVentRBVTask = &taskTable[8];
    Actuators::stopIgniterEnableTask = &taskTable[9];
    Actuators::stopTwoWayTask = &taskTable[10];
    Actuators::stopFuelGemsTask = &taskTable[12];
    Actuators::stopLoxGemsTask = &taskTable[13];

    DEBUGLN("hullo"); //sorry minh

    HAL::initHAL();
    Comms::initComms();
    Actuators::initActuators();
    EReg::initEReg();
    Toggles::initToggles();

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



