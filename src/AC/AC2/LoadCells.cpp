#include "LoadCells.h"

namespace LoadCells {
    float loadCell0Value;
    float loadCell1Value;
    float loadCellSum;
    float lastLoadCellTime;

    Comms::Packet eregAbortPacket = { .id = HAL::eregAbortID };
    Comms::Packet generalAbortPacket = { .id = HAL::generalAbortID };

    uint8_t hysteresisValue = 0;

    void initLoadCells() {
        HAL::lcAmp0.set_scale(3889); // TODO: re-calibrate
        HAL::lcAmp1.set_scale(-3941); // TODO: re-calibrate

        HAL::lcAmp0.tare();
        HAL::lcAmp1.tare();
    }

    uint32_t sampleLoadCells() {
        loadCell0Value = HAL::lcAmp0.get_units(); // in pounds
        loadCell1Value = HAL::lcAmp1.get_units(); // in pounds
        loadCellSum = loadCell0Value + loadCell1Value;

        DEBUGF("LC0 VALUE: %f \t LC1 VALUE: %f \n", loadCell0Value, loadCell1Value);

        Comms::Packet tmp = {.id = 120};
        Comms::packetAddFloat(&tmp, loadCell0Value);
        Comms::packetAddFloat(&tmp, loadCell1Value);
        Comms::packetAddFloat(&tmp, loadCell0Value + loadCell1Value);
        Comms::emitPacket(&tmp);
        return samplePeriod;
    }

    uint32_t checkForLCAbort() {
        if (loadCellSum < loadCellThreshold && millis() - lastLoadCellTime < 25) {
            hysteresisValue += 1;
            if (hysteresisValue >= hysteresisThreshold) {
                abortAll();
            }
        } else {
            hysteresisValue = 0;
        }

        return samplePeriod;
    }

    bool abortAll() {
        uint8_t[4] ip_addresses = [HAL::loxTankEndIp, HAL::loxInjectorEndIp, HAL::fuelTankEndIp, HAL::fuelInjectorEndIp];
        for (uint8_t ip : ip_addresses)
            Comms::emitPacket(&eregAbortPacket, ip);
    }
};
