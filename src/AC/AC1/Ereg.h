#pragma once

#include "Arduino.h"
#include "../Comms.h"
#include "../ERegBoard.h"
#include "Automation.h"

namespace EReg {

    extern float fuelTankPTValue;
    extern float loxTankPTValue;
    extern float fuelInjectorPTValue;
    extern float loxInjectorPTValue;

    void initEReg();

    void abort();
    void startFlow();
    void interpretTelemetry(Comms::Packet packet, uint8_t ip);
   
}