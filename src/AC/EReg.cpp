#include "Ereg.h"

namespace EReg {

    char packetBuffer[sizeof(Comms::Packet)];

    float hpPT = 0;
    float lpPT = 0;
    float injectorPT = 0;
    float motorAngle = 0;

    void initEReg() {
        // EReg board connected to Serial8 of AC Teensy
        Serial8.begin(115200);

        Comms::registerCallback(1, zero);
        Comms::registerCallback(2, setPressureSetpoint);
        Comms::registerCallback(3, flow);
        Comms::registerCallback(4, stopFlow);
        Comms::registerCallback(5, setPIDConstants);
        Comms::registerCallback(6, abort);
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

    uint32_t sampleTelemetry() {
        Comms::Packet tmp = {.id = 85};
        Comms::packetAddFloat(&tmp, hpPT);
        Comms::packetAddFloat(&tmp, lpPT);
        Comms::packetAddFloat(&tmp, injectorPT);
        Comms::packetAddFloat(&tmp, motorAngle);
        Comms::emitPacket(&tmp);

        return samplePeriod;
    }

    void sendToEReg(Comms::Packet packet) {
        Serial8.write(packet->id);
        Serial8.write(packet->len);
        Serial8.write(packet->timestamp, 4);
        Serial8.write(packet->checksum, 2);
        Serial8.write(packet->data, packet->len);
        Serial8.write('\n');
    }

}