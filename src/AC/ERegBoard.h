#pragma once

#include "Arduino.h"
#include "Comms.h"

/**
 * @brief Abstraction of the ethernet communication to and from an EReg board
 * 
 */

class ERegBoard {

    private:
    uint8_t id_;

    public:
    uint8_t ip_address;
    ERegBoard(uint8_t ip_address, uint8_t id);
    void sendEthernet(Comms::Packet *packet, uint8_t ip_address);
    uint8_t getID();
};