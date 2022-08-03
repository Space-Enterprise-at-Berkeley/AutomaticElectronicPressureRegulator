#include "Ereg.h"

namespace EReg {

    char packetBuffer[sizeof(Comms::Packet)];

    float hpPT = 0;
    float lpPT = 0;
    float injectorPT = 0;
    float motorAngle = 0;

    void initEReg() {
        // EReg board connected to Serial8 of AC Teensy
        Serial8.begin(9600);

        Comms::registerCallback(1, zero);
        Comms::registerCallback(2, setPressureSetpoint);
        Comms::registerCallback(3, flow);
        Comms::registerCallback(4, fullOpen);
    }

    Comms::Packet eRegZeroPacket = {.id = 1};
    void zero(Comms::Packet tmp) {
        eRegZeroPacket.data[0] = tmp.data[0];
        sendToEReg(eRegZeroPacket);
    }
    
    Comms::Packet eRegPressureSetpointPacket = {.id = 2};
    void setPressureSetpoint(Comms::Packet tmp) {
        for (int i = 0; i < 4; i++) {
            eRegPressureSetpointPacket.data[i] = tmp.data[i];
        }
        sendToEReg(eRegPressureSetpointPacket);
    }

    Comms::Packet eRegFlowPacket = {.id = 3};
    void flow(Comms::Packet tmp) {
        eRegFlowPacket.data[0] = tmp.data[0];
        sendToEReg(eRegFlowPacket);
    }

    Comms::Packet eRegFullOpenPacket = {.id = 4};
    void fullOpen(Comms::Packet tmp) {
        sendToEReg(eRegFlowPacket);
    }

    uint32_t sampleEregReadingsTask() {
        Comms::Packet tmp = {.id = 85};
        Comms::packetAddFloat(&tmp, hpPT);
        Comms::packetAddFloat(&tmp, lpPT);
        Comms::packetAddFloat(&tmp, injectorPT);
        Comms::packetAddFloat(&tmp, motorAngle);
        Comms::emitPacket(&tmp);

        return samplePeriod;
    }

}