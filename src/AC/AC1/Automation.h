#pragma once

#include "Actuators.h"
#include "Ereg.h"

#include <Arduino.h>
#include <stdlib.h>

namespace Automation {

    const float autoVentUpperThreshold = 700; //TODO actual value 650 - 620
    const float autoVentLowerThreshold = 650;

    extern Task *flowTask; // perform burn
    extern Task *abortFlowTask;
    extern Task *autoventFuelTask;
    extern Task *autoventLoxTask;

    void initAutomation(Task *flowTask, Task *abortFlowTask, Task *autoventFuelTask, Task *autoventLoxTask);
    void beginFlow(Comms::Packet packet, uint8_t ip);
    uint32_t flow();
    void beginManualAbortFlow(Comms::Packet packet, uint8_t ip);
    void beginAbortFlow();
    uint32_t abortFlow();
    uint32_t autoventFuelGemValveTask();
    uint32_t autoventLoxGemValveTask();

};

#define STATE_ACTIVATE_TWO_WAY 0
#define STATE_EREG_BEGIN 1
#define STATE_EREG_ABORT 2
#define STATE_MANUAL_ABORT 3
#define STATE_END_FLOW 4

