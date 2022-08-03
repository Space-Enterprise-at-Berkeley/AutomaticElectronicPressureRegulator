#pragma once

#include "Arduino.h"
#include "Comms.h"

namespace ERegComms {
    void sendToEReg(Comms::Packet *packet);
    uint32_t receiveFromEreg();
}
