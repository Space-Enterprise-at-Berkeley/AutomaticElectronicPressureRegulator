#pragma once

#include <Common.h>

#include "HAL.h"
#include "Comms.h"

#include <Arduino.h>
#include <INA219.h>

namespace Toggles {
    
    extern uint32_t toggleCheckPeriod; // interval for checking valve current and voltages

    const float maxValveCurrent = 1.0;

    extern float ctl12vChan1Voltage;
    extern float arctl12vChan1Current;
    const uint8_t ctl12vChan1Pin = HAL::ctl12vChan1;

    extern float ctl12vChan2Voltage;
    extern float ctl12vChan2Current;
    const uint8_t ctl12vChan2Pin = HAL::ctl12vChan2;

    extern float igniterVoltage;
    extern float igniterCurrent;
    const uint8_t igniterPin = HAL::ctl24vChan1;

    extern float twoWayVoltage;
    extern float twoWayCurrent;
    const uint8_t twoWayPin = HAL::ctl24vChan2;
    
    void initToggles();

    void sampleToggle(Comms::Packet *packet, INA219 *ina, float *voltage, float *current);
    uint32_t ctl12vChan1Sample();
    uint32_t ctl12vChan2Sample();
    uint32_t igniterSample();
    uint32_t twoWaySample();
};