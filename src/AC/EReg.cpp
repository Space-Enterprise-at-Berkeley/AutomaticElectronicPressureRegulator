#include "Ereg.h"

namespace EReg {

    uint32_t samplePeriod = 12.5 * 1000; // 80 Hz
    //char packetBuffer[sizeof(Comms::Packet)];
    char packetBuffer2[1000];
    int packetBuffer2Ctr = 0;

    std::map<uint8_t, Comms::commFunction> eregCallbackMap;

    Comms::Packet eregStartFlowPacket = {.id = 0};
    Comms::Packet eregAbortPacket = {.id = 1};
    Comms::Packet eregSetPositionPacket = {.id = 2};
    Comms::Packet eregPressurizePacket = {.id = 3};
    Comms::Packet eregDiagnosticPacket = {.id = 4};

    Comms::Packet mainGroundStationPacket = {.id = 0};

    void initEReg() {
        // EReg board connected to Serial8 of AC Teensy
        Serial8.begin(115200);
        Comms::registerCallback(34, runDiagnostic);
        // Comms::registerCallback(0, startLoxFlow);
        // Comms::registerCallback(1, startFuelFlow);
        // Comms::registerCallback(2, startFlow);
        // Comms::registerCallback(3, abort);
        // Comms::registerCallback(4, setLoxPosition);
        // Comms::registerCallback(5, setFuelPosition);
        // Comms::registerCallback(6, staticPressurizeLox);
        // Comms::registerCallback(6, staticPressurizeFuel);
        // Comms::registerCallback(6, activateIgniter);
        registerEregCallback(0, interpretTelemetry);

    }


    void registerEregCallback(uint8_t id, Comms::commFunction function) {
        eregCallbackMap.insert(std::pair<int, Comms::commFunction>(id, function));
    }

    void evokeCallbackFunction(Comms::Packet *packet, uint8_t ip) {
        uint16_t checksum = *(uint16_t *)&packet->checksum;
        if (checksum == Comms::computePacketChecksum(packet)) {
            if(eregCallbackMap.count(packet->id)) {
                eregCallbackMap.at(packet->id)(*packet, ip);
            }
        }
    }

    long start = micros();
    bool startedDiagnostic = false;

    void interpretTelemetry(Comms::Packet packet, uint8_t ip) {
        // DEBUG("Received ereg telem packet with id: ");
        // DEBUG(packet.id);
        // DEBUG("\n");
        // DEBUGLN(Comms::packetGetFloat(&packet, 0));
        // DEBUGLN(Comms::packetGetFloat(&packet, 4));
        // DEBUGLN(Comms::packetGetFloat(&packet, 8));
        // DEBUGLN(Comms::packetGetFloat(&packet, 12));
        // DEBUGLN(Comms::packetGetFloat(&packet, 24));
        if (packet.len > 0) {
            Serial.printf("packet id %d with len %d: 12:%f, 16: %f, 20: %f\n", packet.id, packet.len, Comms::packetGetFloat(&packet, 12),
            Comms::packetGetFloat(&packet, 16), Comms::packetGetFloat(&packet, 20));
        }
        //Serial.printf("id: %d, len: %d\n", packet.id, packet.len);
        if (packet.id == 0) { //remap packet 0 (ereg POV) to packet 5 (dashboard/AC POV). Everything else is the same.
            packet.id = 5;
        }
        if (packet.len > 0) {
            Comms::emitPacket(&packet);
        } else if (packet.len < 0) {
            Serial.println("wtf");
        }

        // if (micros() - start > 5e6 && !startedDiagnostic) {
        //     DEBUGLN("SENT TELEMETRY EREG");
        //     sendToEReg(&eregDiagnosticPacket); 
        //     startedDiagnostic = true;
        // }
    }

    void runDiagnostic() {
        sendToEReg(&eregDiagnosticPacket);
    }

    void startLoxFlow(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregStartFlowPacket);
        //TODO send to different serial buses
    }

    void startFuelFlow(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregStartFlowPacket);
    }

    void startFlow(Comms::Packet tmp, uint8_t ip) {
        startLoxFlow(tmp, ip);
        //TODO add delay
        startFuelFlow(tmp, ip);
    }

    void abort(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregAbortPacket);
    }

    void setLoxPosition(Comms::Packet tmp, uint8_t ip) {
        Comms::packetAddFloat(&eregSetPositionPacket, Comms::packetGetFloat(&tmp, 0));
        sendToEReg(&eregSetPositionPacket);
        std::fill_n(eregSetPositionPacket.data, sizeof(float), 0);
    }

    void setFuelPosition(Comms::Packet tmp, uint8_t ip) {
        Comms::packetAddFloat(&eregSetPositionPacket, Comms::packetGetFloat(&tmp, 0));
        sendToEReg(&eregSetPositionPacket);
        std::fill_n(eregSetPositionPacket.data, sizeof(float), 0);
        //TODO differentiate to different serial ports
    }

    void staticPressurizeLox(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregPressurizePacket);
    }

    void staticPressurizeFuel(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregPressurizePacket);
    }

    void activateIgniter(Comms::Packet tmp, uint8_t ip) {

    }

    //TODO split this up for each serial port, can pass in serial bus
    uint32_t sampleTelemetry() {


        while (Serial8.available()) {
            packetBuffer2[packetBuffer2Ctr] = Serial8.read();
            packetBuffer2Ctr++;
            int p = packetBuffer2Ctr - 1;

            if ((eregCallbackMap.count(packetBuffer2[p])) && (p > 3) && (packetBuffer2[p-1]==0x70) && (packetBuffer2[p-2]==0x69) && (packetBuffer2[p-3]==0x68)) {
                Comms::Packet *packet = (Comms::Packet*) &packetBuffer2; 
                packetBuffer2[0] = packetBuffer2[p];
                packetBuffer2Ctr = 1;
                if (packet->len < 300) {
                    evokeCallbackFunction(packet, Comms::Udp.remoteIP()[3]);
                }
            }
        }

        return samplePeriod;
    }

    void sendToEReg(Comms::Packet *packet) {

        uint32_t timestamp = millis();
        packet->timestamp[0] = timestamp & 0xFF;
        packet->timestamp[1] = (timestamp >> 8) & 0xFF;
        packet->timestamp[2] = (timestamp >> 16) & 0xFF;
        packet->timestamp[3] = (timestamp >> 24) & 0xFF;

        uint16_t checksum = computePacketChecksum(packet);
        packet->checksum[0] = checksum & 0xFF;
        packet->checksum[1] = checksum >> 8;

        Serial8.write(packet->id);
        Serial8.write(packet->len);
        Serial8.write(packet->timestamp, 4);
        Serial8.write(packet->checksum, 2);
        //Serial.printf("%x%x\n", packet->checksum[0], packet->checksum[1]);
        Serial8.write(packet->data, packet->len);
        Serial8.write(0x68);
        Serial8.write(0x69);
        Serial8.write(0x70);
 
    }

}