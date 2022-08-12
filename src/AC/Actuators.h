#pragma once

#include <Common.h>

#include "HAL.h"
#include "Comms.h"

#include <Arduino.h>
#include <INA219.h>

namespace Actuators {
    
    extern uint32_t heaterCheckPeriod; // interval for checking valve current and voltages

    const float maxValveCurrent = 1.0;

    // ac1
    extern float propTankVentRBVVoltage;
    extern float propTankVentRBVCurrent;
    extern uint8_t propTankVentRBVState;
    extern Task *stopPropTankVentRBVTask;
    const uint8_t propTankVentRBVPin1 = HAL::hBrg1Pin1;
    const uint8_t propTankVentRBVPin2 = HAL::hBrg1Pin2;

    // act2
    extern float loxTankVentRBVVoltage;
    extern float loxTankVentRBVCurrent;
    extern uint8_t loxTankVentRBVState;
    extern Task *stopLoxTankVentRBVTask;
    const uint8_t loxTankVentRBVPin1 = HAL::hBrg2Pin1;
    const uint8_t loxTankVentRBVPin2 = HAL::hBrg2Pin2;

    // act3
    extern float propFillRBVVoltage;
    extern float propFillRBVCurrent;
    extern uint8_t propFillRBVState;
    extern Task *stopPropFillRBVTask;
    const uint8_t propFillRBVPin1 = HAL::hBrg3Pin1;
    const uint8_t propFillRBVPin2 = HAL::hBrg3Pin2;

    // act4
    extern float loxFillRBVVoltage;
    extern float loxFillRBVCurrent;
    extern uint8_t loxFillRBVState;
    extern Task *stopLoxFillRBVTask;
    const uint8_t loxFillRBVPin1 = HAL::hBrg4Pin1;
    const uint8_t loxFillRBVPin2 = HAL::hBrg4Pin2;

    // TODO: Set correct OC limits
    const uint8_t OClimits[7] = {4,4,4,4,4,4,4}; //(amps): Act 1, Act 2, Act 3, Act 4, Act 5, Act 6, Act 7
    const float stopCurrent = .1; // Stopped actuator current threshold (amps) 
    
    void extendPropTankVentRBV();
    void retractPropTankVentRBV();
    uint32_t stopPropTankVentRBV();
    void brakePropTankVentRBV();
    void propTankVentRBVPacketHandler(Comms::Packet tmp);

    void extendLoxTankVentRBV();
    void retractLoxTankVentRBV();
    uint32_t stopLoxTankVentRBV();
    void brakeLoxTankVentRBV();
    void loxTankVentRBVPacketHandler(Comms::Packet tmp);

    void extendPropFillRBV();
    void retractPropFillRBV();
    uint32_t stopPropFillRBV();
    void brakePropFillRBV();
    void propFillRBVPacketHandler(Comms::Packet tmp);

    void extendLoxFillRBV();
    void retractLoxFillRBV();
    uint32_t stopLoxFillRBV();
    void brakeLoxFillRBV();
    void loxFillRBVPacketHandler(Comms::Packet tmp);

    void actPacketHandler(Comms::Packet tmp, void (*extend)(), void (*retract)(), Task *stopTask);

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current, uint8_t *actState, uint8_t actuatorID);

    void initActuators();

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current);
    uint32_t propTankVentRBVSample();
    uint32_t loxTankVentRBVSample();
    uint32_t propFillRBVSample();
    uint32_t loxFillRBVSample();
};