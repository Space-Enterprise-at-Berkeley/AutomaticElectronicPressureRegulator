#pragma once

#include "Arduino.h"
#include "Comms.h"

namespace EReg {

    extern uint32_t samplePeriod;

    void initEReg();
    void startLoxFlow(Comms::Packet tmp, uint8_t ip);
    void startFuelFlow(Comms::Packet tmp, uint8_t ip);
    void startFlow(Comms::Packet tmp, uint8_t ip);
    void abort(Comms::Packet tmp, uint8_t ip);
    void setLoxPosition(Comms::Packet tmp, uint8_t ip);
    void setFuelPosition(Comms::Packet tmp, uint8_t ip);
    void staticPressurizeLox(Comms::Packet tmp, uint8_t ip);
    void staticPressurizeFuel(Comms::Packet tmp, uint8_t ip);
    void activateIgniter(Comms::Packet tmp, uint8_t ip);

    uint32_t sampleTelemetry();
    void sendToEReg(Comms::Packet *packet);

    void registerEregCallback(uint8_t id, Comms::commFunction function);
    void evokeCallbackFunction(Comms::Packet *packet, uint8_t ip);
}