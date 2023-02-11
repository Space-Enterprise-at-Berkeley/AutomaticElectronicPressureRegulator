#pragma once

#include "HAL.h"
#include "Comms.h"

#include <Arduino.h>
#include "Config.h"

namespace LoadCells {
    const uint32_t samplePeriod = 50 * 1000;

    extern float loadCell0Value;
    extern float loadCell1Value;
    extern float loadCellSum;
    extern float lastLoadCellTime;

    extern Task *abortLC;
    extern Task *readLC;
        
    const float loadCellThreshold = 40;

    const uint8_t hysteresisThreshold = 10;


    void initLoadCells();

    uint32_t sampleLoadCells();
    uint32_t checkForAbort();
    uint32_t checkForLCAbort();
    void sendLCAbortPackets(); 
};
