#include "LoadCells.h"

namespace LoadCells {
    float loadCell0Value;
    float loadCell1Value;
    float loadCellSum;
    float lastLoadCellTime;

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
        loadCell1Value = HAL::lcAmp1.get_units(); // in pounds
        loadCellSum = loadCell0Value + loadCell1Value;

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
                Comms::emitPacket(&lcAbortPacket, 21);
            }
        } else {
            hysteresisValue = 0;
        }

        return samplePeriod;
    }
};
