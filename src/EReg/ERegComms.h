#pragma once

#include <Arduino.h>

#ifdef DEBUG_MODE
#define DEBUG(val) Serial.print(val)
#else
#define DEBUG(val)
#endif

namespace ERegComms {
    //https://github.com/sstaub/TeensyID/issues/3

    struct Packet {
        uint8_t id;
        uint8_t len;
        uint8_t timestamp[4];
        uint8_t checksum[2];
        uint8_t data[256];
    };

    typedef void (*commFunction)(Packet);

    /**
     * @brief Registers methods to be called when Comms receives a packet with a specific ID.
     * 
     * @param id The ID of the packet associated with a specific command.
     * @param function a pointer to a method that takes in a Packet struct.
     */
    void registerCallback(uint8_t id, commFunction function);

    void processWaitingPackets();

    void packetAddFloat(Packet *packet, float value);
    void packetAddUint8(Packet *packet, uint8_t value);
    void packetAddInt32(Packet *packet, int32_t value);

    float packetGetFloat(Packet *packet, uint8_t index);
    uint32_t packetGetUint32(Packet *packet, uint8_t index);

    /**
     * @brief Sends packet data over ethernet and serial.
     * 
     * @param packet The packet in which the data is stored.
     */
    void emitPacket(Packet *packet);

    uint16_t computePacketChecksum(Packet *packet);
};
