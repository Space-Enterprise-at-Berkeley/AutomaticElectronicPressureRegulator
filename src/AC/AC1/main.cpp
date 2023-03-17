#include <Common.h>
#include "../Comms.h"
#include "Actuators.h"
// #include "Ereg.h"
#include "HAL.h"
#include "Toggles.h"
#include "Automation.h"

#include <Arduino.h>
#include <Wire.h>
#define DEBUG_MUL
#include <SPI.h>

Task taskTable[] = {

    // {Actuators::activateIgniter, 0},
    {Actuators::deactivateIgniter, 0},

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

    //Automation
    {Automation::flow, 0, false},
    {Automation::abortFlow, 0, false},
    {Automation::autoventFuelGemValveTask, 0},
    {Automation::autoventLoxGemValveTask, 0},
    {Automation::checkIgniter, 0},

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
    Actuators::stop1 = &taskTable[7];
    Actuators::stop2 = &taskTable[8];
    Actuators::stop3 = &taskTable[9];
    Actuators::stop4 = &taskTable[10];
    Actuators::stop5 = &taskTable[11];
    Actuators::stop6 = &taskTable[12];
    Actuators::stop7 = &taskTable[13];

    DEBUGF("hullo\n"); //sorry minh

    HAL::initHAL();
    Comms::initComms();
    Actuators::initActuators();
    EReg::initEReg();
    Toggles::initToggles();
    Automation::initAutomation(&taskTable[14], &taskTable[15], &taskTable[16], &taskTable[17]);

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



