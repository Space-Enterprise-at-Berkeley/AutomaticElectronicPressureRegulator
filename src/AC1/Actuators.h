#pragma once

#include "Common.h"

#include "HAL.h"
#include "Comms.h"

#include <Arduino.h>
#include <INA219.h>

namespace Actuators {
    
    extern uint32_t ActuatorCheckPeriod; // interval for checking valve current and voltages

    const float maxValveCurrent = 1.0;

    extern float fuelTankVentRBVVoltage;
    extern float fuelTankVentRBVCurrent;
    extern uint8_t fuelTankVentRBVState;
    extern Task *stopFuelTankVentRBVTask;
    const uint8_t fuelTankVentRBVPin1 = HAL::hBrg1Pin1;
    const uint8_t fuelTankVentRBVPin2 = HAL::hBrg1Pin2;

    extern float loxTankVentRBVVoltage;
    extern float loxTankVentRBVCurrent;
    extern uint8_t loxTankVentRBVState;
    extern Task *stopLoxTankVentRBVTask;
    const uint8_t loxTankVentRBVPin1 = HAL::hBrg2Pin1;
    const uint8_t loxTankVentRBVPin2 = HAL::hBrg2Pin2;

    extern float igniterEnableVoltage;
    extern float igniterEnableCurrent;
    extern uint8_t igniterEnableState;
    extern Task *stopIgniterEnableTask;
    const uint8_t igniterEnablePin1 = HAL::hBrg3Pin1;
    const uint8_t igniterEnablePin2 = HAL::hBrg3Pin2;

    extern float twoWayVoltage;
    extern float twoWayCurrent;
    extern uint8_t twoWayState;
    extern Task *stopTwoWayTask;
    const uint8_t twoWayPin1 = HAL::hBrg4Pin1;
    const uint8_t twoWayPin2 = HAL::hBrg4Pin2;

    extern float act5Voltage;
    extern float act5Current;
    extern uint8_t act5State;
    extern Task *stop5;
    const uint8_t act5Pin1 = HAL::hBrg5Pin1;
    const uint8_t act5Pin2 = HAL::hBrg5Pin2;

    extern float fuelGemsVoltage;
    extern float fuelGemsCurrent;
    extern uint8_t fuelGemsState;
    extern Task *stopFuelGemsTask;
    const uint8_t fuelGemsPin1 = HAL::hBrg6Pin1;
    const uint8_t fuelGemsPin2 = HAL::hBrg6Pin2;

    extern float loxGemsVoltage;
    extern float loxGemsCurrent;
    extern uint8_t loxGemsState;
    extern Task *stopLoxGemsTask;
    const uint8_t loxGemsPin1 = HAL::hBrg7Pin1;
    const uint8_t loxGemsPin2 = HAL::hBrg7Pin2;

    // TODO: Set correct OC limits
    const uint8_t OClimits[7] = {4,4,4,4,4,4,4}; //(amps): Act 1, Act 2, Act 3, Act 4, Act 5, Act 6, Act 7
    const float stopCurrent = .1; // Stopped actuator current threshold (amps) 
    
    void extendFuelTankVentRBV();
    void retractFuelTankVentRBV();
    uint32_t stopFuelTankVentRBV();
    void brakeFuelTankVentRBV();
    void fuelTankVentRBVPacketHandler(Comms::Packet tmp);

    void extendLoxTankVentRBV();
    void retractLoxTankVentRBV();
    uint32_t stopLoxTankVentRBV();
    void brakeLoxTankVentRBV();
    void loxTankVentRBVPacketHandler(Comms::Packet tmp);

    void extendIgniterRelay();
    void retractIgniterRelay();
    uint32_t stopIgniterRelay();
    void brakeIgniterRelay();
    void igniterRelayPacketHandler(Comms::Packet tmp);

    void extendTwoWay();
    void retractTwoWay();
    uint32_t stopTwoWay();
    void brakeTwoWay();
    void twoWayPacketHandler(Comms::Packet tmp);

    void extendAct5();
    void retractAct5();
    uint32_t stopAct5();
    void brakeAct5();
    void act5PacketHandler(Comms::Packet tmp);

    void extendFuelGems();
    void retractFuelGems();
    uint32_t stopFuelGems();
    void brakeFuelGems();
    void fuelGemsPacketHandler(Comms::Packet tmp);

    void extendLoxGems();
    void retractLoxGems();
    uint32_t stopLoxGems();
    void brakeLoxGems();
    void loxGemsPacketHandler(Comms::Packet tmp);

    void actPacketHandler(Comms::Packet tmp, void (*extend)(), void (*retract)(), Task *stopTask);

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current, uint8_t *actState, uint8_t actuatorID);

    void initActuators();

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current);
    uint32_t fuelTankVentRBVSample();
    uint32_t loxTankVentRBVSample();
    uint32_t igniterEnableSample();
    uint32_t twoWaySample();
    uint32_t act5Sample();
    uint32_t fuelGemsSample();
    uint32_t loxGemsSample();
};