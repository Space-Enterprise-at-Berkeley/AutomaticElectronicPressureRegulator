#pragma once

#include "Arduino.h"
#include "Comms.h"

namespace EReg {

    uint32_t samplePeriod = 12.5 * 1000; // 80 Hz

    void initEReg();
    void zero(Comms::Packet tmp);
    void setPressureSetpoint(Comms::Packet tmp);
    void regulation(Comms::Packet tmp);
    void flow(Comms::Packet tmp);
    void fullOpen(Comms::Packet tmp);

    uint32_t sampleEregReadingsTask();
    void sendToEReg(Comms::Packet packet);
    uint32_t receiveFromEreg();
}