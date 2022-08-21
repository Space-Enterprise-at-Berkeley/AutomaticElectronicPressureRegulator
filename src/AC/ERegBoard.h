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
    HardwareSerial *serial_;
    Comms::Packet failPacket_;

    public:
    char* packetBuffer_;
    int packetBufferCtr_;
    ERegBoard(HardwareSerial *serial, uint8_t id);
    void sendSerial(Comms::Packet *packet);
    Comms::Packet *receiveSerial();
    uint8_t getID();
    uint32_t goodPackets_;
    uint32_t cumPackets_;
};