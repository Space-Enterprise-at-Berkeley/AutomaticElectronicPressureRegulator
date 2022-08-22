#include "LoadCells.h"

namespace LoadCells {
    float loadCell0Value;
    float loadCell1Value;

    void initLoadCells() {
        HAL::lcAmp0.set_scale(3889); // TODO: re-calibrate
        HAL::lcAmp1.set_scale(-3941); // TODO: re-calibrate

        HAL::lcAmp0.tare();
        HAL::lcAmp1.tare();
    }

    uint32_t sampleLoadCells() {
        loadCell0Value = HAL::lcAmp0.get_units(); // in pounds
        loadCell1Value = HAL::lcAmp1.get_units(); // in pounds

        Comms::Packet tmp = {.id = 120};
        Comms::packetAddFloat(&tmp, loadCell0Value);
        Comms::packetAddFloat(&tmp, loadCell1Value);
        Comms::packetAddFloat(&tmp, loadCell0Value + loadCell1Value);
        Comms::emitPacket(&tmp);
        return samplePeriod;
    }
};
