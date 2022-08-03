#include "ERegComms.h"

void sendToEReg(Comms::Packet *packet) {
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
    //send to EReg board
    Serial8.write(packet->id);
    Serial8.write(packet->len);
    Serial8.write(packet->timestamp, 4);
    Serial8.write(packet->checksum, 2);
    Serial8.write(packet->data, packet->len);
    Serial8.write('\n');   
}

uint32_t receiveFromEreg() {
    if(Serial.available()) {
        int cnt = 0;
        while(Serial.available() && cnt < sizeof(Comms::Packet)) {
            packetBuffer[cnt] = Serial.read();
            cnt++;
        }
        Comms::Packet *packet = (Comms::Packet *)&packetBuffer;
    }
    return samplePeriod;
}