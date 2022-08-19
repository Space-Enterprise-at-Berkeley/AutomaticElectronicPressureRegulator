#include "Ereg.h"

namespace EReg {

    uint32_t samplePeriod = 12.5 * 1000; // 80 Hz

    std::map<uint8_t, Comms::commFunction> eregCallbackMap;

    Comms::Packet eregStartFlowPacket = {.id = 0};
    Comms::Packet eregAbortPacket = {.id = 1};
    Comms::Packet eregSetEncoderPositionPacket = {.id = 2};
    Comms::Packet eregPressurizeStaticPacket = {.id = 3};
    Comms::Packet eregRunDiagnosticPacket = {.id = 4};
    Comms::Packet eregZeroEncoderPacket = {.id = 5};
    Comms::Packet eregActuateMainValve = {.id = 6};

    Comms::Packet tempPacket = {.id = 255}; //packet to store the received serial telemetry in.
    //Added because we cast the serial buffer pointer to a packet pointer, so if we overwrite 
    //the packet fields the serial buffer will also get overwritten (change id/timestamp/checksum)

    Comms::Packet mainTelemetryPacket = {.id = 0};
    Comms::Packet mainValveStatePacket = {.id = 1};
    Comms::Packet configTelemetryPacket = {.id = 2};

    Comms::Packet diagnosticPacket = {.id = 12};
    Comms::Packet commandFailPacket = {.id = 13};

    ERegBoard fuelBoard(Serial5, 0);
    ERegBoard loxBoard(Serial8, 1);

    ERegBoard *eregBoards[2] = { &fuelBoard, &loxBoard };

    void initEReg() {

        Comms::registerCallback(1, startOneSidedFlow);
        Comms::registerCallback(2, startFlow);
        Comms::registerCallback(3, abort);
        Comms::registerCallback(4, setERegEncoderPosition);
        Comms::registerCallback(5, actuateMainValve);
        //packet ID 6
        Comms::registerCallback(7, staticPressurize);
        Comms::registerCallback(8, zeroEReg);
        Comms::registerCallback(9, runDiagnostic);

        registerEregCallback(0, interpretTelemetry);
        registerEregCallback(1, interpretMainValves);
        registerEregCallback(2, interpretConfigTelemetry);

        registerEregCallback(12, interpretDiagnosticTelemetry);
        registerEregCallback(13, interpretCommandFailTelemetry);
    }


    void registerEregCallback(uint8_t id, Comms::commFunction function) {
        eregCallbackMap.insert(std::pair<int, Comms::commFunction>(id, function));
    }

    void evokeERegCallbackFunction(Comms::Packet *packet, uint8_t id) {
        uint16_t checksum = *(uint16_t *)&packet->checksum;
        if (checksum == Comms::computePacketChecksum(packet)) {
            // DEBUGF("PACket id %d with len %d: 12:%f, 16: %f, 20: %f\n", packet->id, packet->len, Comms::packetGetFloat(packet, 12),
                        // Comms::packetGetFloat(packet, 16), Comms::packetGetFloat(packet, 20));
            if(eregCallbackMap.count(packet->id)) {
                eregCallbackMap.at(packet->id)(*packet, id);
            }
        } else {
            DEBUGF("BADket id %d with len %d. Expected check: %d, received check: %d\n", packet->id, packet->len, Comms::computePacketChecksum(packet), packet->checksum);
            Comms::dumpPacket(packet);
        }
    }

    void interpretTelemetry(Comms::Packet packet, uint8_t id) {
        if (packet.len > 0) {
            // DEBUGF("packet id %d with len %d: 12:%f, 16: %f, 20: %f\n", packet.id, packet.len, Comms::packetGetFloat(&packet, 12),
            // Comms::packetGetFloat(&packet, 16), Comms::packetGetFloat(&packet, 20));
            uint8_t oldid  = mainTelemetryPacket.id;
            memcpy(&mainTelemetryPacket, &packet, sizeof(Comms::Packet));
            mainTelemetryPacket.id = oldid;
            Comms::packetAddUint8(&configTelemetryPacket, id);
            Comms::emitPacket(&mainTelemetryPacket);
        }
    }

    void interpretMainValves(Comms::Packet packet, uint8_t id) {
        if (packet.len > 0) {
            DEBUGF("received mainvalve packet with id %d, length %d. First uint8_t of payload (should be mainvalve state): %d\n", packet.id, packet.len, packet.data[0]);
            uint8_t oldid = mainValveStatePacket.id;
            memcpy(&mainValveStatePacket, &packet, sizeof(Comms::Packet));
            mainValveStatePacket.id = oldid;
            Comms::packetAddUint8(&configTelemetryPacket, id);
            Comms::emitPacket(&mainValveStatePacket);
        }
    }

    void interpretConfigTelemetry(Comms::Packet packet, uint8_t id) {
        if (packet.len > 0) {
            DEBUGF("received config telemetry packet\n");
            uint8_t oldid = configTelemetryPacket.id; 
            memcpy(&configTelemetryPacket, &packet, sizeof(Comms::Packet));
            configTelemetryPacket.id = oldid;
            Comms::packetAddUint8(&configTelemetryPacket, id);
            Comms::emitPacket(&configTelemetryPacket);
        }
    }
    
    void interpretDiagnosticTelemetry(Comms::Packet packet, uint8_t id) {
        diagnosticPacket.len = 0;
        Comms::packetAddUint8(&diagnosticPacket, packet.data[0]);
        Comms::packetAddUint8(&diagnosticPacket, packet.data[1]);
        Comms::packetAddUint8(&commandFailPacket, id);
        Comms::emitPacket(&diagnosticPacket);
    }

    void interpretCommandFailTelemetry(Comms::Packet packet, uint8_t id) {
        commandFailPacket.len = 0;
        Comms::packetAddUint8(&commandFailPacket, packet.data[0]);
        Comms::packetAddUint8(&commandFailPacket, id);
        Comms::emitPacket(&commandFailPacket);
    }

    void runDiagnostic(Comms::Packet tmp, uint8_t ip) {
        fuelBoard.sendSerial(eregRunDiagnosticPacket);
        loxBoard.sendSerial(eregRunDiagnosticPacket);
    }

    void startOneSidedFlow(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        eregBoards[i]->sendSerial(eregStartFlowPacket);
    }

    void actuateMainValve(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        eregBoards[i]->sendSerial(eregActuateMainValve);
    }

    void startFlow(Comms::Packet tmp, uint8_t ip) {
        //TODO start automation sequence
    }

    void abort(Comms::Packet tmp, uint8_t ip) {
        //TODO implement individual aborts?
        fuelBoard.sendSerial(eregAbortPacket);
        loxBoard.sendSerial(eregAbortPacket);
    }

    void setERegEncoderPosition(Comms::Packet tmp, uint8_t ip) {
        eregSetEncoderPositionPacket.len = 0;
        Comms::packetAddFloat(&eregSetEncoderPositionPacket, Comms::packetGetFloat(&tmp, 1));

        int i = tmp.data[0];
        eregBoards[i]->sendSerial(eregSetEncoderPositionPacket);
    }

    void staticPressurize(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        eregBoards[i]->sendSerial(eregPressurizeStaticPacket);
    }

    void zeroEReg(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        eregBoards[i]->sendSerial(eregZeroEncoderPacket); 
    }

    uint32_t sampleFuelEregTelemetry() {
        sampleTelemetry(fuelBoard);
        return samplePeriod;
    }

    uint32_t sampleLoxEregTelemetry() {
        sampleTelemetry(loxBoard);
        return samplePeriod;
    }

    void sampleTelemetry(ERegBoard board) {
        Comms::Packet packet = board.receiveSerial();
        
        if (packet.id != 255) { //TODO figure out what to return when no packet
            evokeERegCallbackFunction(&packet, board.getID());
        }
    }

}