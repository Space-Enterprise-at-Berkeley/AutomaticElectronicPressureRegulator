#pragma once

#include "Arduino.h"
#include "../Comms.h"
#include "../ERegBoard.h"

namespace EReg {

    extern uint32_t samplePeriod;
    extern float fuelTankPTValue;
    extern float loxTankPTValue;

    void initEReg();
    void startLOXFlow(Comms::Packet tmp, uint8_t ip);
    void startFuelFlow(Comms::Packet tmp, uint8_t ip);
    void startOneSidedFlow(Comms::Packet tmp, uint8_t ip);
    void startFlow(Comms::Packet tmp, uint8_t ip);
    void abort(Comms::Packet tmp, uint8_t ip);

    void runDiagnostic(Comms::Packet tmp, uint8_t ip);
    void setERegEncoderPosition(Comms::Packet tmp, uint8_t ip);
    void staticPressurize(Comms::Packet tmp, uint8_t ip);
    void actuateMainValve(Comms::Packet tmp, uint8_t ip);
    void zeroEReg(Comms::Packet tmp, uint8_t ip);

    void interpretTelemetry(Comms::Packet packet, uint8_t id);
    void interpretMainValves(Comms::Packet packet, uint8_t id);
    void interpretConfigTelemetry(Comms::Packet packet, uint8_t id);
    void interpretDiagnosticTelemetry(Comms::Packet packet, uint8_t id);
    void interpretCommandFailTelemetry(Comms::Packet packet, uint8_t id);

    /**
     * @brief Copies contents of src packet to dst packet
     * 
     * @param dst 
     * @param src 
     * @param id id of dst packet
     */
    void packetcpy(Comms::Packet *dst, Comms::Packet *src, uint8_t id);

    uint32_t sampleFuelEregTelemetry();
    uint32_t sampleLoxEregTelemetry();
    void sampleTelemetry(ERegBoard board);

    void registerEregCallback(uint8_t id, Comms::commFunction function);
    void evokeERegCallbackFunction(Comms::Packet *packet, uint8_t board);
}