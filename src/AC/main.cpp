#include <Common.h>
#include "Comms.h"
#include "Actuators.h"
#include "Power.h"
#include "Ereg.h"
#include "HAL.h"
#include "Toggles.h"

#include <Arduino.h>
#include <Wire.h>
#define DEBUG_MUL
#include <SPI.h>

Task taskTable[] = {
    // actuators
    {Actuators::act1Sample, 0},
    {Actuators::act2Sample, 0},
    {Actuators::act3Sample, 0},
    {Actuators::act4Sample, 0},
    {Actuators::act5Sample, 0},
    {Actuators::act6Sample, 0},
    {Actuators::act7Sample, 0},

    {Actuators::stopAct1, 0, false},
    {Actuators::stopAct2, 0, false},
    {Actuators::stopAct3, 0, false},
    {Actuators::stopAct4, 0, false},
    {Actuators::stopAct5, 0, false},
    {Actuators::stopAct6, 0, false},
    {Actuators::stopAct7, 0, false},

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
    Actuators::stop1 = &taskTable[7];
    Actuators::stop2 = &taskTable[8];
    Actuators::stop3 = &taskTable[9];
    Actuators::stop4 = &taskTable[10];
    Actuators::stop5 = &taskTable[11];
    Actuators::stop6 = &taskTable[12];
    Actuators::stop7 = &taskTable[13];

    DEBUGLN("Starting AC");

    HAL::initHAL();
    Comms::initComms();
    Actuators::initActuators();
    // Power::initPower();
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
