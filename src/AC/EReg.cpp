#include "Ereg.h"

namespace EReg {

    uint32_t samplePeriod = 12.5 * 1000; // 80 Hz
    //char packetBuffer[sizeof(Comms::Packet)];
    char packetBuffer2[1000];
    int packetBuffer2Ctr = 0;

    std::map<uint8_t, Comms::commFunction> eregCallbackMap;

    Comms::Packet eregStartFlowPacket = {.id = 0};
    Comms::Packet eregAbortPacket = {.id = 1};
    Comms::Packet eregSetEncoderPositionPacket = {.id = 2};
    Comms::Packet eregPressurizeStaticPacket = {.id = 3};
    Comms::Packet eregRunDiagnosticPacket = {.id = 4};
    Comms::Packet eregZeroEncoderPacket = {.id = 5};
    Comms::Packet eregActuateMainValve = {.id = 6};

    Comms::Packet tempPacket = {.id = -1}; //packet to store the received serial telemetry in.
    //Added because we cast the serial buffer pointer to a packet pointer, so if we overwrite 
    //the packet fields the serial buffer will also get overwritten (change id/timestamp/checksum)
    
    

    Comms::Packet mainTelemetryPacket = {.id = 0};

    void initEReg() {
        // EReg board connected to Serial8 of AC Teensy
        Serial8.begin(115200);

        
        Comms::registerCallback(1, startOneSidedFlow);
        Comms::registerCallback(2, startFlow);
        Comms::registerCallback(3, abort);
        Comms::registerCallback(4, setERegEncoderPosition);
        Comms::registerCallback(5, actuateMainValve);
        Comms::registerCallback(6, staticPressurize);
        //packet ID 7
        Comms::registerCallback(8, zeroEReg);
        Comms::registerCallback(9, runDiagnostic);



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
        } else {

            DEBUG("packet checksum")
        }
    }

    long start = micros();
    bool startedDiagnostic = false;

    void interpretTelemetry(Comms::Packet packet, uint8_t ip) {

        if (packet.len > 0) {
            DEBUGF("packet id %d with len %d: 12:%f, 16: %f, 20: %f\n", packet.id, packet.len, Comms::packetGetFloat(&packet, 12),
            Comms::packetGetFloat(&packet, 16), Comms::packetGetFloat(&packet, 20));
        }
        if (packet.id == 0) { //remap packet 0 (ereg POV) to packet 5 (dashboard/AC POV). Everything else is the same.
            uint8_t oldid  = mainTelemetryPacket.id;
            memcpy(&mainTelemetryPacket, &packet, sizeof(Comms::Packet));
            mainTelemetryPacket.id = oldid;
        }
        if (mainTelemetryPacket.len > 0) {
            Comms::emitPacket(&mainTelemetryPacket);
        } else if (mainTelemetryPacket.len <= 0) {
            Serial.println("wtf");
        }

    }

    void runDiagnostic(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregRunDiagnosticPacket);
    }

    void startLoxFlow(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregStartFlowPacket);
        //TODO send to different serial buses
    }

    void startFuelFlow(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregStartFlowPacket);
    }

    void startOneSidedFlow(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        if (i == 0) {
            startFuelFlow(tmp, ip);
        } else if (i == 1) {
            startLoxFlow(tmp, ip);
        } else {
            Serial.println("packet data must be either 0 (Fuel) or 1 (LOX) <-- startOneSidedFlow");
        }
    }

    void actuateMainValve(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        if (i == 0) {
            actuateFuelMainValve(tmp, ip);
        } else if (i == 1) {
            actuateLOXMainValve(tmp, ip);
        } else {
            Serial.println("packet data must be either 0 (Fuel) or 1 (LOX) <-- actuateMainValve");
        } 

    }

    void actuateFuelMainValve(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregActuateMainValve); 
    }

    void actuateLOXMainValve(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregActuateMainValve);
    }

    void startFlow(Comms::Packet tmp, uint8_t ip) {
        startLoxFlow(tmp, ip);
        //TODO add delay
        //TODO add igniter fire, BW, current, other checks
        startFuelFlow(tmp, ip);
    }

    void abort(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregAbortPacket);
    }

    void setERegEncoderPosition(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        if (i == 0) {
            setFuelPosition(tmp, ip);
        } else if (i == 1) {
            setLoxPosition(tmp, ip);
        } else {
            Serial.println("packet data must be either 0 (Fuel) or 1 (LOX) <-- setERegEncoderPosition");
        }
        
    }

    void setLoxPosition(Comms::Packet tmp, uint8_t ip) {
        Comms::packetAddFloat(&eregSetEncoderPositionPacket, Comms::packetGetFloat(&tmp, 1));
        sendToEReg(&eregSetEncoderPositionPacket);
        std::fill_n(eregSetEncoderPositionPacket.data, sizeof(float), 0); 
    }

    void setFuelPosition(Comms::Packet tmp, uint8_t ip) {
        Comms::packetAddFloat(&eregSetEncoderPositionPacket, Comms::packetGetFloat(&tmp, 1));
        sendToEReg(&eregSetEncoderPositionPacket);
        std::fill_n(eregSetEncoderPositionPacket.data, sizeof(float), 0);
        //TODO differentiate to different serial ports
    }

    void staticPressurize(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        if (i == 0) {
            staticPressurizeFuel(tmp, ip);
        } else if (i == 1) {
            staticPressurizeLox(tmp, ip);
        } else {
            Serial.println("packet data must be either 0 (Fuel) or 1 (LOX) <-- staticPressurize");
        } 
    }
    void staticPressurizeLox(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregPressurizeStaticPacket);
    }

    void staticPressurizeFuel(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregPressurizeStaticPacket);
    }

    void zeroEReg(Comms::Packet tmp, uint8_t ip) {
        int i = tmp.data[0];
        if (i == 0) {
            zeroFuelERegEncoder(tmp, ip);
        } else if (i == 1) {
            zeroLOXERegEncoder(tmp, ip);
        } else {
            Serial.println("packet data must be either 0 (Fuel) or 1 (LOX) <-- zeroEReg");
        }  
    }
    void zeroFuelERegEncoder(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregZeroEncoderPacket);
    }

    void zeroLOXERegEncoder(Comms::Packet tmp, uint8_t ip) {
        sendToEReg(&eregZeroEncoderPacket);
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
                //Comms::Packet *packet = (Comms::Packet*) &packetBuffer2; 
                memcpy(&tempPacket, &packetBuffer2, sizeof(Comms::Packet));
                Comms::Packet *packet = &tempPacket;
                packetBuffer2[0] = packetBuffer2[p];
                packetBuffer2Ctr = 1;
                evokeCallbackFunction(packet, Comms::Udp.remoteIP()[3]);
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