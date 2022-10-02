#pragma once

#include "Arduino.h"
#include "Comms.h"

/**
 * @brief Abstraction of the serial communication to and from an EReg board
 * 
 */

class ERegBoard {

    private:
    uint8_t id_;
    Comms::Packet failPacket_;
    uint32_t EREG_BOARD_BAUD_RATE = 500000;

    public:
    char* packetBuffer_;
    int packetBufferCtr_;
    uint8_t ip_address;
    ERegBoard(uint8_t ip_address, uint8_t id);
    void sendEthernet(Comms::Packet *packet, uint8_t ip_address);
    uint8_t getID();
    uint32_t goodPackets_;
    uint32_t cumPackets_;
};