#include "Thermocouples.h"

namespace Thermocouples {
    uint32_t tcUpdatePeriod = 50 * 1000;
    const uint32_t tcAbortRefreshPeriod = 1000UL * 1000UL; // 2s delay between abort sends
    Comms::Packet readingPacket = {.id = 110};
    Comms::Packet tcAbortPacket = {.id = 30};
    float tempBuffer[Config::numberOfTC][Config::tempBufferSize];
    int buffer_i;
    MAX31855* amps[] = {&HAL::tcAmp0, &HAL::tcAmp1, &HAL::tcAmp2};

    void initThermocouples() {
        buffer_i = 0;
    }

    uint32_t checkForAbort() {
        //check thermocouple temperatures to be below a threshold        
        for (int tcNumber = 0; tcNumber < Config::numberOfTC; tcNumber++) {
            int aboveThresholdCount = 0;

            for(int i = 0; i < Config::tempBufferSize; i++){
                // DEBUGF("%f ", tempBuffer[tcNumber][i]);
                if (tempBuffer[tcNumber][i] >= Config::temperatureThreshold) {
                    aboveThresholdCount++;
                }
            }
            // DEBUG(" | ");

            if (aboveThresholdCount >= Config::tempNumberThreshold) {
                DEBUGF("Temperature abort triggered by TC %d!\n", tcNumber);
                // Comms::emitPacket(&tcAbortPacket);
                tcAbortPacket.len = 0;
                sendTCAbortPackets();
                return tcAbortRefreshPeriod;
            }
        }
        
        // DEBUGLN("");
        return tcUpdatePeriod;
    }

    void sendTCAbortPackets() {
        Comms::Packet abortMessage = {.id = 30, .len = 0};
        // Comms::emitDirectedPacket(&abortMessage, FUEL_TANK_EREG_ADDR);
        // Comms::emitDirectedPacket(&abortMessage, FUEL_INJECTOR_EREG_ADDR);
        // Comms::emitDirectedPacket(&abortMessage, LOX_TANK_EREG_ADDR);
        // Comms::emitDirectedPacket(&abortMessage, LOX_INJECTOR_EREG_ADDR);
        Comms::emitDirectedPacket(&abortMessage, AC_EREG_ADDR);
    }

    uint32_t sendTCReadingPacket(){
        // DEBUGLN("TC readings ");
        readingPacket.len = 0;
        for(int i = 0; i < Config::numberOfTC; i++){
            tempBuffer[i][buffer_i] = amps[i]->readCelsius();
            Comms::packetAddFloat(&readingPacket, tempBuffer[i][buffer_i]);            
            // DEBUG(" " + String(tempBuffer[i][buffer_i]));
        }
        // DEBUGLN("");
        buffer_i = (buffer_i + 1) % Config::tempBufferSize;
        Comms::emitPacket(&readingPacket);
        return tcUpdatePeriod;

    }
    

};
