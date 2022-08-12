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

    // act5
    extern float twoWayVoltage;
    extern float twoWayCurrent;
    extern uint8_t twoWayState;
    extern Task *stopTwoWayTask;
    const uint8_t twoWayPin1 = HAL::hBrg5Pin1;
    const uint8_t twoWayPin2 = HAL::hBrg5Pin2;

    // act6
    extern float loxGemsVoltage;
    extern float loxGemsCurrent;
    extern uint8_t loxGemsState;
    extern Task *stopLoxGemsTask;
    const uint8_t loxGemsPin1 = HAL::hBrg6Pin1;
    const uint8_t loxGemsPin2 = HAL::hBrg6Pin2;

    // act7
    extern float igniterVoltage;
    extern float igniterCurrent;
    extern uint8_t igniterState;
    extern Task *stopIgniterTask;
    const uint8_t igniterPin1 = HAL::hBrg7Pin1;
    const uint8_t igniterPin2 = HAL::hBrg7Pin2;

    // TODO: Set correct OC limits
    const uint8_t OClimits[7] = {4,4,4,4,4,4, 4}; //(amps): Act 1, Act 2, Act 3, Act 4, Act 5, Act 6, Act 7
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

    void extendTwoWay();
    void retractTwoWay();
    uint32_t stopTwoWay();
    void brakeTwoWay();
    void twoWayPacketHandler(Comms::Packet tmp);

    void extendLoxGems();
    void retractLoxGems();
    uint32_t stopLoxGems();
    void brakeLoxGems();
    void loxGemsPacketHandler(Comms::Packet tmp);

    void extendIgniter();
    void retractIgniter();
    uint32_t stopIgniter();
    void brakeIgniter();
    void igniterPacketHandler(Comms::Packet tmp);

    void actPacketHandler(Comms::Packet tmp, void (*extend)(), void (*retract)(), Task *stopTask);

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current, uint8_t *actState, uint8_t actuatorID);

    void initActuators();

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current);
    uint32_t propTankVentRBVSample();
    uint32_t loxTankVentRBVSample();
    uint32_t propFillRBVSample();
    uint32_t loxFillRBVSample();
    uint32_t twoWaySample();
    uint32_t loxGemsSample();
    uint32_t igniterSample();

};