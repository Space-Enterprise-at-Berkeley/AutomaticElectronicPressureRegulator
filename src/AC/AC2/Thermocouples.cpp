#include "Thermocouples.h"

namespace Thermocouples {
    uint32_t tcUpdatePeriod = 100 * 1000;
    Comms::Packet tcPacket;

    float tc0Value;
    float tc1Value;
    float tc2Value;
    float tc3Value;
    float tc4Value;

    void initThermocouples() {
    }

    uint32_t tcSample(MAX31855 *amp, uint8_t packetID, float *value) {
        // read from all TCs in sequence
        // *value = amp->readThermocouple();
        
        // tcPacket.id = packetID;
        // tcPacket.len = 0;
        // Comms::packetAddFloat(&tcPacket, *value);
        
        // Comms::emitPacket(&tcPacket);
        // // return the next execution time
        // return tcUpdatePeriod;

        *value = amp->readCelsius();

        tcPacket.id = packetID;
        tcPacket.len = 0;
        Comms::packetAddFloat(&tcPacket, *value);
        Comms::emitPacket(&tcPacket);

        return tcUpdatePeriod;
    }

    uint32_t tc0Sample() { return tcSample(&HAL::tcAmp0, 110, &tc0Value); }
    uint32_t tc1Sample() { return tcSample(&HAL::tcAmp1, 111, &tc1Value); }
    uint32_t tc2Sample() { return tcSample(&HAL::tcAmp2, 112, &tc2Value); }
    uint32_t tc3Sample() { return tcSample(&HAL::tcAmp3, 113, &tc3Value); }
    uint32_t tc4Sample() { return tcSample(&HAL::tcAmp4, 114, &tc4Value); }

};
