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
    // {Actuators::act1Sample, 0},
    // {Actuators::act2Sample, 0},
    // {Actuators::act3Sample, 0},
    // {Actuators::act4Sample, 0},
    // {Actuators::act5Sample, 0},
    // {Actuators::act6Sample, 0},
    // {Actuators::act7Sample, 0}, //#6


    //Thermocouples
    {&Thermocouples::checkForAbort, (uint32_t) 0, true},
    {&Thermocouples::sendTCReadingPacket, (uint32_t) 0, true},
    //Load Cells
    {&LoadCells::checkForLCAbort, (uint32_t) 0, true},
    {&LoadCells::sampleLoadCells, (uint32_t) 0, true},
    // thermocouples
    // {Thermocouples::tc0Sample, 0},
    // {Thermocouples::tc1Sample, 0},
    // {Thermocouples::tc2Sample, 0},
    // {Thermocouples::tc3Sample, 0},
    // {Thermocouples::tc4Sample, 0},

};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

int main() {
    setup();
    loop();
    return 0;
}

void setup(){
    // hardware setup
    Serial.begin(115200);
    #ifdef DEBUG_MODE
    while(!Serial) {} // wait for user to open serial port (debugging only)
    #endif

    Thermocouples::abortTC = &taskTable[0];
    Thermocouples::readTC = &taskTable[1];
    LoadCells::abortLC = &taskTable[2];
    LoadCells::readLC = &taskTable[3];

    DEBUGLN("hullo from AC2");

    HAL::initHAL();
    DEBUGLN("iniitiated Hal");
    Comms::initComms();
    DEBUGLN("ethernet started fr");
    Actuators::initActuators(); 
    DEBUGLN("after init actuators");
    LoadCells::initLoadCells();
    Thermocouples::initThermocouples();
    DEBUGLN("done with initialization");
}

void loop(){
    while(1) {
        uint32_t ticks = micros(); // current time in microseconds
        for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
            if (taskTable[i].enabled && taskTable[i].nexttime - ticks > UINT32_MAX / 2) {
                DEBUGLN("running task " + String(i));
                taskTable[i].nexttime = ticks + taskTable[i].taskCall();
            }
            else {
                DEBUGLN("skipped task " + String(i));
            }
        }
        DEBUGLN("finished looping over all tasks");
        // Comms::processWaitingPackets();
    }
}
