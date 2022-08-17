#include "Comms.h"

namespace Comms {

    
    #ifdef DEBUG_MODE
    String inString_ = "";
    #endif

    commFunction callbackMap[numIDs] = {};
    char packetBuffer[buf_size];
    unsigned int bufferIndex = 0;

    void initComms() {
        Serial.begin(38400);
    }

    void registerCallback(uint8_t id, commFunction function) {
        callbackMap[id] = function;
    }

    /**
     * @brief Checks checksum of packet and tries to call the associated callback function.
     * 
     * @param packet Packet to be processed.
     */
    void evokeCallbackFunction(Packet *packet) {
        uint16_t checksum = *(uint16_t *)&packet->checksum;
        Serial.println(computePacketChecksum(packet));
        if (checksum == computePacketChecksum(packet)) {
            if(packet->id < numIDs) {
                callbackMap[packet->id](*packet);
            }
        }
    }

    void processWaitingPackets() {
        #ifdef DEBUG_MODE
        if(Serial.available()) {
            while (Serial.available()) {
                int inChar = Serial.read();
                if (isDigit(inChar) || inChar=='-') {
                    inString_ += (char)inChar;
                }
                if (inChar == '\n') {
                    Packet packet = {.id = inString_.toInt()};
                    uint16_t checksum = computePacketChecksum(&packet);
                    packet.checksum[0] = checksum & 0xFF;
                    packet.checksum[1] = checksum >> 8;
                    inString_ = "";
                    DEBUG("Mocking inbound packet with id ");
                    DEBUGLN(packet.id);
                    evokeCallbackFunction(&packet);
                    return;
                }
            }
        }
        #else
        if(Serial.available()) {

            if (bufferIndex >= buf_size) {
                bufferIndex = 0;
            }
            
            while(Serial.available()) {
                packetBuffer[bufferIndex] = Serial.read();
                bufferIndex++;
                int lastWrittenIndex = bufferIndex - 1;
                if (
                    (lastWrittenIndex >= 3) && 
                    (packetBuffer[lastWrittenIndex] == 0x70) &&
                    (packetBuffer[lastWrittenIndex - 1] == 0x69) &&
                    (packetBuffer[lastWrittenIndex - 2] == 0x68)
                ) {
                    Packet *packet = (Packet *)&packetBuffer;
                    bufferIndex = 0;
                    Serial.print("packet id: ");
                    Serial.print(packet->id);
                    Serial.print("\tlen: ");
                    Serial.print(packet->len);
                    Serial.print(" \t check");
                    Serial.println(packet->checksum[0]);
                    evokeCallbackFunction(packet);
                }
            }
        }
        #endif
    }

    void packetAddFloat(Packet *packet, float value) {
        uint32_t rawData = * ( uint32_t * ) &value;
        packet->data[packet->len] = rawData & 0xFF;
        packet->data[packet->len + 1] = rawData >> 8 & 0xFF;
        packet->data[packet->len + 2] = rawData >> 16 & 0xFF;
        packet->data[packet->len + 3] = rawData >> 24 & 0xFF;
        packet->len += 4;
    }

    void packetAddUint32(Packet *packet, uint32_t value) {
        packet->data[packet->len] = value & 0xFF;
        packet->data[packet->len + 1] = value >> 8 & 0xFF;
        packet->data[packet->len + 2] = value >> 16 & 0xFF;
        packet->data[packet->len + 3] = value >> 24 & 0xFF;
        packet->len += 4;
    }

    void packetAddUint16(Packet *packet, uint16_t value) {
        packet->data[packet->len] = value & 0xFF;
        packet->data[packet->len + 1] = value >> 8 & 0xFF;
        packet->len += 2;
    }

    void packetAddUint8(Packet *packet, uint8_t value) {
        packet->data[packet->len] = value;
        packet->len++;
    }

    /**
     * Inserts string into specified packet.
     * IMPORTANT: String must come after all other data, and there cannot be 2 strings in one packet
     * This is because the parser will treat all bytes from start of string to end of packet as part of the string
     * 
     * @param packet pointer to packet to which message should be appended
     * @param message string message. If this is too long to fit within the packet, it will be truncated
     */
    void packetAddString(Packet *packet, String message) {
        // unsigned int messageLength = min(message.length(), payloadSize - packet->len);
        // for (unsigned int i = 0; i<messageLength; i++) {
        //     packet->data[i + packet->len] = message[i];
        // }
        char message2[] = "hullo"; //len=5
        for (int i = 0; i < 5; i++) {
            packet->data[i] = message2[i];
        }
        packet->len += 5;
    }

    float packetGetFloat(Packet *packet, uint8_t index) {
        uint32_t rawData = packet->data[index+3];
        rawData <<= 8;
        rawData += packet->data[index+2];
        rawData <<= 8;
        rawData += packet->data[index+1];
        rawData <<= 8;
        rawData += packet->data[index];
        return * ( float * ) &rawData;
    }

    uint32_t packetGetUint32(Packet *packet, uint8_t index) {
        uint32_t rawData = packet->data[index+3];
        rawData <<= 8;
        rawData += packet->data[index+2];
        rawData <<= 8;
        rawData += packet->data[index+1];
        rawData <<= 8;
        rawData += packet->data[index];
        return rawData;
    }

    uint32_t packetGetUint8(Packet *packet, uint8_t index) {
        return packet->data[index];
    }

    /**
     * @brief Sends packet to both groundstations.
     * 
     * @param packet Packet to be sent.
     */
    void emitPacket(Packet *packet) {
        //add timestamp to struct
        uint32_t timestamp = millis();
        packet->timestamp[0] = timestamp & 0xFF;
        packet->timestamp[1] = (timestamp >> 8) & 0xFF;
        packet->timestamp[2] = (timestamp >> 16) & 0xFF;
        packet->timestamp[3] = (timestamp >> 24) & 0xFF;

        //calculate and append checksum to struct
        uint16_t checksum = computePacketChecksum(packet);
        packet->checksum[0] = checksum & 0xFF;
        packet->checksum[1] = checksum >> 8;

        Serial.write(packet->id);
        Serial.write(packet->len);
        Serial.write(packet->timestamp, 4);
        Serial.write(packet->checksum, 2);
        Serial.write(packet->data, packet->len);
        Serial.write(0x68);
        Serial.write(0x69);
        Serial.write(0x70);
    }

    /**
     * @brief generates a 2 byte checksum from the information of a packet
     * 
     * @param data pointer to data array
     * @param len length of data array
     * @return uint16_t 
     */
    uint16_t computePacketChecksum(Packet *packet) {

        uint8_t sum1 = 0;
        uint8_t sum2 = 0;

        sum1 = sum1 + packet->id;
        sum2 = sum2 + sum1;
        sum1 = sum1 + packet->len;
        sum2 = sum2 + sum1;
        
        for (uint8_t index = 0; index < 4; index++) {
            sum1 = sum1 + packet->timestamp[index];
            sum2 = sum2 + sum1;
        }

        for (uint8_t index = 0; index < packet->len; index++) {
            sum1 = sum1 + packet->data[index];
            sum2 = sum2 + sum1;
        }
        return (((uint16_t)sum2) << 8) | (uint16_t) sum1;
    }
};
