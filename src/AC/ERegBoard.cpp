#include "ERegBoard.h"


ERegBoard::ERegBoard(uint8_t ip_address_, uint8_t id) {
    id_ = id;
    ip_address = ip_address_;
}

void ERegBoard::sendEthernet(Comms::Packet *packet, uint8_t ip) {
    Comms::emitPacket(packet, ip);
}

uint8_t ERegBoard::getID() {
    return id_;
}