#include "Ereg.h"

namespace EReg {

    uint32_t samplePeriod = 12.5 * 1000; // 80 Hz
    float fuelTankPTValue = 0;
    float loxTankPTValue = 0;

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

    Comms::Packet fuelMainTelemetryPacket = {.id = 0};
    Comms::Packet loxMainTelemetryPacket = {.id = 1};
    Comms::Packet fuelMainValveStateFuelPacket = {.id = 2};
    Comms::Packet loxMainValveStatePacket = {.id = 3};
    Comms::Packet fuelConfigTelemetryPacket = {.id = 4};
    Comms::Packet loxConfigTelemetryPacket = {.id = 5};

    Comms::Packet fuelDiagnosticPacket = {.id = 12};
    Comms::Packet loxDiagnosticPacket = {.id = 13};
    Comms::Packet fuelCommandFailPacket = {.id = 14};
    Comms::Packet loxCommandFailPacket = {.id = 15};
    Comms::Packet fuelFlowStatePacket = {.id = 16};
    Comms::Packet loxFlowStatePacket = {.id = 17};

    ERegBoard fuelBoard(&Serial5, 0);
    ERegBoard loxBoard(&Serial8, 1);

    ERegBoard *eregBoards[2] = { &fuelBoard, &loxBoard };

    uint32_t lastTime = millis();


    void initEReg() {
        Comms::registerCallback(1, startOneSidedFlow);
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
        registerEregCallback(14, interpretFlowStateTelemetry);

        Comms::registerCallback(21, resetEreg);
    }


    void registerEregCallback(uint8_t id, Comms::commFunction function) {
        eregCallbackMap.insert(std::pair<int, Comms::commFunction>(id, function));
    }

    void packetcpy(Comms::Packet *dst, Comms::Packet *src, uint8_t id) {
        memcpy(dst, src, sizeof(Comms::Packet));
        dst->id = id;
    }

    void evokeERegCallbackFunction(Comms::Packet *packet, uint8_t id) {
        uint16_t checksum = *(uint16_t *)&packet->checksum;
        eregBoards[id]->cumPackets_++;
        if (checksum == Comms::computePacketChecksum(packet)) {
             //DEBUGF("PACket id %d with len %d: 12:%f, 16: %f, 20: %f\n", packet->id, packet->len, Comms::packetGetFloat(packet, 12),
                        //Comms::packetGetFloat(packet, 16), Comms::packetGetFloat(packet, 20));
            if(eregCallbackMap.count(packet->id)) {
                eregCallbackMap.at(packet->id)(*packet, id);
            }
            eregBoards[id]->goodPackets_ ++;
        } else {
                //uint16_t expectedCheck = Comms::computePacketChecksum(packet);
                // DEBUGF("BADket id %d with len %d. received check: 0x%x%x\n", packet->id, packet->len,
                //    packet->checksum[1], packet->checksum[0]);
                // Comms::dumpPacket(packet);
            }

        }
    

    void interpretTelemetry(Comms::Packet packet, uint8_t id) {
        if (packet.len > 0) {
            //DEBUGF("packet id %d with len %d: 12:%f, 16: %f, 20: %f\n", packet.id, packet.len, Comms::packetGetFloat(&packet, 12),
            // Comms::packetGetFloat(&packet, 16), Comms::packetGetFloat(&packet, 20));
            if (id == 0) {
                fuelTankPTValue = Comms::packetGetFloat(&packet, 4);
                packetcpy(&fuelMainTelemetryPacket, &packet, fuelMainTelemetryPacket.id);
                Comms::emitPacket(&fuelMainTelemetryPacket);
            } else if (id == 1) {
                loxTankPTValue = Comms::packetGetFloat(&packet, 4);
                packetcpy(&loxMainTelemetryPacket, &packet, loxMainTelemetryPacket.id);
                Comms::emitPacket(&loxMainTelemetryPacket);
            }
        }
    }

    void interpretMainValves(Comms::Packet packet, uint8_t id) {
        if (packet.len > 0) {
            // DEBUGF("received mainvalve packet with id %d, length %d. First uint8_t of payload (should be mainvalve state): %d\n", packet.id, packet.len, packet.data[0]);
            if (id == 0) {
                packetcpy(&fuelMainValveStateFuelPacket, &packet, fuelMainValveStateFuelPacket.id);
                Comms::emitPacket(&fuelMainValveStateFuelPacket);
            } else if (id == 1) {
                packetcpy(&loxMainValveStatePacket, &packet, loxMainValveStatePacket.id);
                Comms::emitPacket(&loxMainValveStatePacket);
            }
        }
    }

    void interpretConfigTelemetry(Comms::Packet packet, uint8_t id) {
        if (packet.len > 0) {
            DEBUGLN("received config telemetry packet");
            if (id == 0) {
                packetcpy(&fuelConfigTelemetryPacket, &packet, fuelConfigTelemetryPacket.id);
                Comms::emitPacket(&fuelConfigTelemetryPacket);
            } else if (id == 1) {
                packetcpy(&loxConfigTelemetryPacket, &packet, loxConfigTelemetryPacket.id);
                Comms::emitPacket(&loxConfigTelemetryPacket);
            }
        }
    }
    
    void interpretDiagnosticTelemetry(Comms::Packet packet, uint8_t id) {
            if (id == 0) {
                packetcpy(&fuelDiagnosticPacket, &packet, fuelDiagnosticPacket.id);
                Comms::emitPacket(&fuelDiagnosticPacket);
            } else if (id == 1) {
                packetcpy(&loxDiagnosticPacket, &packet, loxDiagnosticPacket.id);
                Comms::emitPacket(&loxDiagnosticPacket);
            }
    }

    void interpretCommandFailTelemetry(Comms::Packet packet, uint8_t id) {
            DEBUG("RECEIVED Command Fail WITH ID: ");
            DEBUGLN(Comms::packetGetUint8(&packet, 0));
            if (id == 0) {
                packetcpy(&fuelCommandFailPacket, &packet, fuelCommandFailPacket.id);
                Comms::emitPacket(&fuelCommandFailPacket);
            } else if (id == 1) {
                packetcpy(&loxCommandFailPacket, &packet, loxCommandFailPacket.id);
                Comms::emitPacket(&loxCommandFailPacket);
            }
    }

    void interpretFlowStateTelemetry(Comms::Packet packet, uint8_t id) {
        uint8_t state = packet.data[0];
        DEBUG("Recieved flow state: ");
        DEBUGLN(state);
        if (id == 0) {
            packetcpy(&fuelFlowStatePacket, &packet, fuelFlowStatePacket.id);
            Comms::emitPacket(&fuelFlowStatePacket);
        } else if (id == 1) {
            packetcpy(&loxFlowStatePacket, &packet, loxFlowStatePacket.id);
            Comms::emitPacket(&loxFlowStatePacket);
        }

        //check for abort
        if (state == 0) {
            Automation::beginAbortFlow();
        }
    }

    void runDiagnostic(Comms::Packet tmp, uint8_t ip) {
        fuelBoard.sendSerial(&eregRunDiagnosticPacket);
        loxBoard.sendSerial(&eregRunDiagnosticPacket);
    }

    void startOneSidedFlow(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        eregBoards[i]->sendSerial(&eregStartFlowPacket);
    }

    void actuateMainValve(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        eregActuateMainValve.len = 0;
        Comms::packetAddUint8(&eregActuateMainValve, tmp.data[1]);
        eregBoards[i]->sendSerial(&eregActuateMainValve);
    }

    void startFlow() {
        eregBoards[0]->sendSerial(&eregStartFlowPacket);
        //TODO start both boards
    }

    void abort() {
        //TODO implement individual aborts?
        fuelBoard.sendSerial(&eregAbortPacket);
        loxBoard.sendSerial(&eregAbortPacket);
    }

    void resetEreg(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        eregBoards[i]->sendSerial(&eregAbortPacket);
    }

    void setERegEncoderPosition(Comms::Packet tmp, uint8_t ip) {
        eregSetEncoderPositionPacket.len = 0;
        Comms::packetAddFloat(&eregSetEncoderPositionPacket, Comms::packetGetFloat(&tmp, 1));

        int i = tmp.data[0];
        eregBoards[i]->sendSerial(&eregSetEncoderPositionPacket);
    }

    void staticPressurize(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        eregBoards[i]->sendSerial(&eregPressurizeStaticPacket);
    }

    void zeroEReg(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        eregBoards[i]->sendSerial(&eregZeroEncoderPacket); 
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

        if ((millis() - lastTime) > 1000) {
            lastTime = millis();
            DEBUGF("in last second: fuel percent %f, lox percent %f, fuel good %d, lox good %d, cumPackets %d\n", ((float)eregBoards[0]->goodPackets_ / (float)eregBoards[0]->cumPackets_),
                                                        ((float)eregBoards[1]->goodPackets_ / (float)eregBoards[1]->cumPackets_), fuelBoard.goodPackets_, loxBoard.goodPackets_, fuelBoard.cumPackets_);
            eregBoards[0]->goodPackets_ = 0;
            eregBoards[1]->goodPackets_ = 0;
            eregBoards[0]->cumPackets_ = 0;
            eregBoards[1]->cumPackets_ = 0;
        }
        Comms::Packet *packet = board.receiveSerial();
        if (packet->id != 255) { //TODO figure out what to return when no packet
            evokeERegCallbackFunction(packet, board.getID());
            //DEBUGF("Recieved telemetry from board %i \n", board.getID();)
        }
    }

}

