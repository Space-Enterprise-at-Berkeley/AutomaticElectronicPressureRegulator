
#include "HAL.h"
#include "Comms.h"

#include <Arduino.h>
#include "Config.h"

#pragma once

namespace Thermocouples {
    extern uint32_t tcUpdatePeriod;

    void initThermocouples();
    uint32_t checkForAbort();
    void sendTCAbortPackets(); 
    uint32_t sendTCReadingPacket();
};
