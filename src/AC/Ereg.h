#pragma once

#include "Arduino.h"
#include "Comms.h"

namespace EReg {

    uint32_t samplePeriod = 12.5 * 1000; // 80 Hz

    void initEReg();
    void zero(Comms::Packet tmp);
    void setPressureSetpoint(Comms::Packet tmp);
    void flow(Comms::Packet tmp);
    void stopFlow(Comms::Packet tmp);
    void setPIDConstants(Comms::Packet tmp);
    void abort(Comms::Packet tmp);

    uint32_t sampleTelemetry();
    void sendToEReg(Comms::Packet packet);
}