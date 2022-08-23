#include "ERegBoard.h"


ERegBoard::ERegBoard(HardwareSerial *serial, uint8_t id) {
    id_ = id;

    serial_ = serial;
    serial_->begin(38400);

    packetBuffer_ = new char[sizeof(Comms::Packet) + 3];
    packetBufferCtr_ = 0;
    
    cumPackets_ = 0;
    goodPackets_ = 0; //lol

    failPacket_ = {.id = 255};
}

void ERegBoard::sendSerial(Comms::Packet *packet) {
    uint32_t timestamp = millis();
    packet->timestamp[0] = timestamp & 0xFF;
    packet->timestamp[1] = (timestamp >> 8) & 0xFF;
    packet->timestamp[2] = (timestamp >> 16) & 0xFF;
    packet->timestamp[3] = (timestamp >> 24) & 0xFF;

    uint16_t checksum = computePacketChecksum(packet);
    packet->checksum[0] = checksum & 0xFF;
    packet->checksum[1] = checksum >> 8;

    serial_->write(packet->id);
    serial_->write(packet->len);
    serial_->write(packet->timestamp, 4);
    serial_->write(packet->checksum, 2);
    serial_->write(packet->data, packet->len);
    serial_->write(0x68);
    serial_->write(0x69);
    serial_->write(0x70);
}

Comms::Packet *ERegBoard::receiveSerial() {
    if (serial_->available()) {
        packetBufferCtr_ = 0;
        while (packetBufferCtr_ < 3 || !(packetBuffer_[packetBufferCtr_-3] == 0x68 &&
            packetBuffer_[packetBufferCtr_-2] == 0x69 && packetBuffer_[packetBufferCtr_-1] == 0x70)) {
            int serialByte = serial_->read();
            if (serialByte == -1) return &failPacket_;
            packetBuffer_[packetBufferCtr_] = serialByte;
            packetBufferCtr_++;
            if (packetBufferCtr_ > sizeof(Comms::Packet) + 3) return &failPacket_;
        }
        Comms::Packet* packet = (Comms::Packet*) &packetBuffer_;
        return packet;
    }
    return &failPacket_;
}

uint8_t ERegBoard::getID() {
    return id_;
}