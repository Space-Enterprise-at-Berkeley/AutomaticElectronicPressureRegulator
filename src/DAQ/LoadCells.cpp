#include "LoadCells.h"

namespace LoadCells {
    float loadCell0Value;
    float loadCell1Value;
    float loadCellSum;
    float lastLoadCellTime;

    Task *abortLC;
    Task *readLC;

    Comms::Packet lcAbortPacket = {.id = 31};

    uint8_t hysteresisValue = 0;

    void initLoadCells() {
        HAL::lcAmp0.set_scale(3889); // TODO: re-calibrate
        HAL::lcAmp1.set_scale(-3941); // TODO: re-calibrate

        HAL::lcAmp0.tare();
        HAL::lcAmp1.tare();
    }

    uint32_t sampleLoadCells() {
        loadCell0Value = HAL::lcAmp0.get_units(); // in pounds
        loadCell1Value = -1 * HAL::lcAmp1.get_units(); // in pounds
        loadCellSum = loadCell0Value + loadCell1Value;

        // DEBUGF("LC0 VALUE: %f \t LC1 VALUE: %f \n", loadCell0Value, loadCell1Value);

        Comms::Packet tmp = {.id = 120};
        Comms::packetAddFloat(&tmp, loadCell0Value);
        Comms::packetAddFloat(&tmp, loadCell1Value);
        Comms::packetAddFloat(&tmp, loadCell0Value + loadCell1Value);
        Comms::emitPacket(&tmp);

        lastLoadCellTime = micros();
        
        return samplePeriod;
    }

    uint32_t checkForLCAbort() {
        // DEBUGF("Checking for LC abort with loadcellsum %f", loadCellSum);
        if (loadCellSum < loadCellThreshold && millis() - lastLoadCellTime < 25) {
            hysteresisValue += 1;
            if (hysteresisValue >= hysteresisThreshold) {
                DEBUGF("LOAD CELL ABORT TRIGGERED \n");
                // Comms::emitPacket(&lcAbortPacket);
                sendLCAbortPackets();
            }
        } else {
            hysteresisValue = 0;
        }

        return samplePeriod;
    }

    void sendLCAbortPackets() {
        Comms::Packet abortMessage = {.id = 31, .len = 0};
        // Comms::emitDirectedPacket(&abortMessage, FUEL_TANK_EREG_ADDR);
        // Comms::emitDirectedPacket(&abortMessage, FUEL_INJECTOR_EREG_ADDR);
        // Comms::emitDirectedPacket(&abortMessage, LOX_TANK_EREG_ADDR);
        // Comms::emitDirectedPacket(&abortMessage, LOX_INJECTOR_EREG_ADDR);
        Comms::emitDirectedPacket(&abortMessage, AC_EREG_ADDR);
    }

};
