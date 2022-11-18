#pragma once

#include <Common.h>

#include "HAL.h"
#include "Comms.h"

#include <Arduino.h>

namespace Actuators {
    
    extern uint32_t actuatorCheckPeriod; // interval for checking valve current and voltages

    enum channelType  {RBV, VALVE};
    const channelType channelTypes[1] = {RBV};

    const float maxValveCurrent = 1.0;

    extern uint8_t act1State;
    extern Task *stop1;
    const uint8_t act1Pin1 = HAL::hBrg1Pin1;
    const uint8_t act1Pin2 = HAL::hBrg1Pin2;

    void extendAct1();
    void retractAct1();
    uint32_t stopAct1();
    void brakeAct1();
    void act1PacketHandler(Comms::Packet tmp);

    void actPacketHandler(Comms::Packet tmp, void (*extend)(), void (*retract)(), Task *stopTask);

    void initActuators();
};