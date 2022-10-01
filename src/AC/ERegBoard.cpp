#include "ERegBoard.h"


ERegBoard::ERegBoard(uint8_t ip_address, uint8_t id) {
    id_ = id;

    // serial_ = serial;
    // serial_->begin(EREG_BOARD_BAUD_RATE);

    packetBuffer_ = new char[sizeof(Comms::Packet) + 3];
    packetBufferCtr_ = 0;
    
    cumPackets_ = 0;
    goodPackets_ = 0; //lol

    failPacket_ = {.id = 255};
}

void ERegBoard::sendEthernet(Comms::Packet *packet, uint8_t ip) {
    Comms::emitPacket(&packet, ip);
}

uint8_t ERegBoard::getID() {
    return id_;
}