#include "Ereg.h"

namespace EReg {

    uint32_t samplePeriod = 12.5 * 1000; // 80 Hz
    char packetBuffer[sizeof(Comms::Packet)];

    std::map<uint8_t, Comms::commFunction> eregCallbackMap;

    Comms::Packet eregStartFlowPacket = {.id = 0};
    Comms::Packet eregAbortPacket = {.id = 1};
    Comms::Packet eregSetPositionPacket = {.id = 2};
    Comms::Packet eregPressurizePacket = {.id = 3};
    Comms::Packet eregDiagnosticPacket = {.id = 4};

    void initEReg() {
        // EReg board connected to Serial8 of AC Teensy
        Serial8.begin(115200);

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
        DEBUG("Received ereg telem packet with id: ");
        DEBUG(packet.id);
        DEBUG("\n");
        DEBUGLN(Comms::packetGetFloat(&packet, 0));
        DEBUGLN(Comms::packetGetFloat(&packet, 4));
        DEBUGLN(Comms::packetGetFloat(&packet, 8));
        DEBUGLN(Comms::packetGetFloat(&packet, 12));
        DEBUGLN(Comms::packetGetFloat(&packet, 24));

        if (micros() - start > 5e6 && !startedDiagnostic) {
            DEBUGLN("SENT TELEMETRY EREG");
            sendToEReg(&eregDiagnosticPacket);
            startedDiagnostic = true;
        }
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
        if(Serial8.available()) {
            int cnt = 0;
            while(Serial8.available() && cnt < sizeof(Comms::Packet)) {
                packetBuffer[cnt] = Serial8.read();
                cnt++;
            }
            Comms::Packet *packet = (Comms::Packet *)&packetBuffer;
            evokeCallbackFunction(packet, Comms::Udp.remoteIP()[3]);
        }
        return samplePeriod;
    }

    void sendToEReg(Comms::Packet *packet) {
        Serial8.write(packet->id);
        Serial8.write(packet->len);
        Serial8.write(packet->timestamp, 4);
        Serial8.write(packet->checksum, 2);
        Serial8.write(packet->data, packet->len);
        Serial8.write('\n');
    }

}