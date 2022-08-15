#pragma once

#include "Arduino.h"
#include "Comms.h"

namespace EReg {

    extern uint32_t samplePeriod;

    void initEReg();
    void startLOXFlow(Comms::Packet tmp, uint8_t ip);
    void startFuelFlow(Comms::Packet tmp, uint8_t ip);
    void startOneSidedFlow(Comms::Packet tmp, uint8_t ip);
    void startFlow(Comms::Packet tmp, uint8_t ip);
    void abort(Comms::Packet tmp, uint8_t ip);

    void runDiagnostic(Comms::Packet tmp, uint8_t ip);
    void setERegEncoderPosition(Comms::Packet tmp, uint8_t ip);
    void setLoxPosition(Comms::Packet tmp, uint8_t ip);
    void setFuelPosition(Comms::Packet tmp, uint8_t ip);
    void staticPressurize(Comms::Packet tmp, uint8_t ip);
    void staticPressurizeLox(Comms::Packet tmp, uint8_t ip);
    void staticPressurizeFuel(Comms::Packet tmp, uint8_t ip);
    void activateIgniter(Comms::Packet tmp, uint8_t ip);
    void actuateMainValve(Comms::Packet tmp, uint8_t ip);
    void actuateFuelMainValve(Comms::Packet tmp, uint8_t ip);
    void actuateLOXMainValve(Comms::Packet tmp, uint8_t ip);
    void zeroEReg(Comms::Packet tmp, uint8_t ip);
    void zeroFuelERegEncoder(Comms::Packet tmp, uint8_t ip);
    void zeroLOXERegEncoder(Comms::Packet tmp, uint8_t ip);

    void interpretTelemetry(Comms::Packet packet, uint8_t ip);

    uint32_t sampleTelemetry();
    void sendToEReg(Comms::Packet *packet);

    void registerEregCallback(uint8_t id, Comms::commFunction function);
    void evokeCallbackFunction(Comms::Packet *packet, uint8_t ip);
}