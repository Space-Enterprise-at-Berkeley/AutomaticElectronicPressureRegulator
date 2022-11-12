#include <Common.h>
#include "Comms.h"
#include "Thermocouples.h"
#include "LoadCells.h"
#include "HAL.h"

#include <Arduino.h>
#include <Wire.h>
#define DEBUG_MUL
#include <SPI.h>

Task taskTable[] = {
    //Thermocouples
    {Thermocouples::checkForAbort, 0, true},
    {Thermocouples::sendTCReadingPacket, 0, true},
    //Load Cells
    {LoadCells::checkForLCAbort, 0, false},
    {LoadCells::sampleLoadCells, 0, true}
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
    Ethernet.init(25);
    #ifdef DEBUG_MODE
    while(!Serial) {} // wait for user to open serial port (debugging only)
    #endif

    // Thermocouples::abortTC = &taskTable[0];
    // Thermocouples::readTC = &taskTable[1];
    LoadCells::abortLC = &taskTable[2];
    LoadCells::readLC = &taskTable[3];

    DEBUGLN("hullo from AC2");

    HAL::initHAL();
    DEBUGLN("iniitiated Hal");
    Comms::initComms();
    DEBUGLN("ethernet started fr");
    LoadCells::initLoadCells();
    Thermocouples::initThermocouples();
    DEBUGLN("done with initialization");
}

void loop(){
    while(1) {
        uint32_t ticks = micros(); // current time in microseconds
        for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
            if (taskTable[i].enabled && taskTable[i].nexttime - ticks > UINT32_MAX / 2) {
                // DEBUGLN("running task " + String(i));
                taskTable[i].nexttime = ticks + taskTable[i].taskCall();
            }
            else {
                // DEBUGLN("skipped task " + String(i));
            }
        }
        // DEBUGLN("finished looping over all tasks");
        Comms::processWaitingPackets();
    }
}
