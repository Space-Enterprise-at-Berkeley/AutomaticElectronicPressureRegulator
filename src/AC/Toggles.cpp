#include "Toggles.h"

namespace Toggles {

    // TODO: change this to appropriate value
    uint32_t toggleCheckPeriod = 50 * 1000;

    // TODO: set correct telem packet IDs
    Comms::Packet ctl12vChan1Packet = {.id = 0};
    float ctl12vChan1Voltage = 0.0;
    float ctl12vChan1Current = 0.0;

    Comms::Packet ctl12vChan2Packet = {.id = 0};
    float ctl12vChan2Voltage = 0.0;
    float ctl12vChan2Current = 0.0;

    Comms::Packet ctl24vChan1Packet = {.id = 0};
    float igniterVoltage = 0.0;
    float igniterCurrent = 0.0;

    Comms::Packet ctl24vChan2Packet = {.id = 0};
    float twoWayVoltage = 0.0;
    float twoWayCurrent = 0.0;

    void sampleToggle(Comms::Packet *packet, INA219 *ina, float *voltage, float *current) {
        *voltage = ina->readBusVoltage();
        *current = ina->readShuntCurrent();

        packet->len = 0;
        Comms::packetAddFloat(packet, *voltage);
        Comms::packetAddFloat(packet, *current);
        Comms::emitPacket(packet);
    }

    uint32_t ctl12vChan1Sample() {
        sampleToggle(&ctl12vChan1Packet, &HAL::chan0, &ctl12vChan1Voltage, &ctl12vChan1Current);
        return toggleCheckPeriod;
    }

    uint32_t ctl12vChan2Sample() {
        sampleToggle(&ctl12vChan2Packet, &HAL::chan1, &ctl12vChan2Voltage, &ctl12vChan2Current);
        return toggleCheckPeriod;
    }

    uint32_t igniterSample() {
        sampleToggle(&ctl24vChan1Packet, &HAL::chan2, &igniterVoltage, &igniterCurrent);
        return toggleCheckPeriod;
    }
    uint32_t twoWaySample() {
        sampleToggle(&ctl24vChan2Packet, &HAL::chan3, &twoWayVoltage, &twoWayCurrent);
        return toggleCheckPeriod;
    }

    void toggleToggle(Comms::Packet packet, uint8_t pin) {
        digitalWrite(pin, packet.data[0]);
    }

    void toggleIgniter(Comms::Packet packet) {
        toggleToggle(packet, igniterPin);
    }

    void toggleTwoWay(Comms::Packet packet) {
        toggleToggle(packet, twoWayPin);
    }

    void initToggles() {
        //todo fill in correct telem
        Comms::registerCallback(toggleIgniter, 182);
        Comms::registerCallback(toggleTwoWay, 183);
    }
};