#include <Common.h>
#include <AC/Comms.h>
#include "Actuators.h"
#include "Thermocouples.h"
#include "LoadCells.h"
#include "HAL.h"

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
    {Actuators::act7Sample, 0}, //#6

    {Actuators::stopAct1, 0, false}, //#7
    {Actuators::stopAct2, 0, false},
    {Actuators::stopAct3, 0, false},
    {Actuators::stopAct4, 0, false},
    {Actuators::stopAct5, 0, false},
    {Actuators::stopAct6, 0, false},
    {Actuators::stopAct7, 0, false}, //#13

    // thermocouples
    // {Thermocouples::tc0Sample, 0},
    // {Thermocouples::tc1Sample, 0},
    // {Thermocouples::tc2Sample, 0},
    // {Thermocouples::tc3Sample, 0},
    // {Thermocouples::tc4Sample, 0},

    // load cells
    // {LoadCells::sampleLoadCells, 0},
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

    DEBUGLN("hullo from AC2");

    HAL::initHAL();
    Comms::initComms();
    Actuators::initActuators();
    DEBUGLN("got here");
    // LoadCells::initLoadCells();
    DEBUGLN("ethernet started fr");

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
