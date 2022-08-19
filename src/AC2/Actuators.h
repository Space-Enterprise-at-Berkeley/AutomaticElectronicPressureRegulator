#pragma once

#include "Common.h"

#include "HAL.h"
#include "Comms.h"

#include <Arduino.h>
#include <INA219.h>

namespace Actuators {
    
    extern uint32_t ActuatorCheckPeriod; // interval for checking valve current and voltages

    const float maxValveCurrent = 1.0;

    extern float fuelFillRBVVoltage;
    extern float fuelFillRBVCurrent;
    extern uint8_t fuelFillRBVState;
    extern Task *stopFuelFillRBVTask;
    const uint8_t fuelFillRBVPin1 = HAL::hBrg1Pin1;
    const uint8_t fuelFillRBVPin2 = HAL::hBrg1Pin2;

    extern float loxFillRBVVoltage;
    extern float loxFillRBVCurrent;
    extern uint8_t loxFillRBVState;
    extern Task *stopLoxFillRBVTask;
    const uint8_t loxFillRBVPin1 = HAL::hBrg2Pin1;
    const uint8_t loxFillRBVPin2 = HAL::hBrg2Pin2;

    extern float pressFillRBVVoltage;
    extern float pressFillRBVCurrent;
    extern uint8_t pressFillRBVState;
    extern Task *stopPressFillRBVTask;
    const uint8_t pressFillRBVPin1 = HAL::hBrg3Pin1;
    const uint8_t pressFillRBVPin2 = HAL::hBrg3Pin2;

    extern float pressLineVentRBVVoltage;
    extern float pressLineVentRBVCurrent;
    extern uint8_t pressLineVentRBVState;
    extern Task *stopPressLineVentRBVTask;
    const uint8_t pressLineVentRBVPin1 = HAL::hBrg4Pin1;
    const uint8_t pressLineVentRBVPin2 = HAL::hBrg4Pin2;

    extern float act5Voltage;
    extern float act5Current;
    extern uint8_t act5State;
    extern Task *stop5;
    const uint8_t act5Pin1 = HAL::hBrg5Pin1;
    const uint8_t act5Pin2 = HAL::hBrg5Pin2;

    extern float act6Voltage;
    extern float act6Current;
    extern uint8_t act6State;
    extern Task *stop6;
    const uint8_t act6Pin1 = HAL::hBrg6Pin1;
    const uint8_t act6Pin2 = HAL::hBrg6Pin2;

    extern float act7Voltage;
    extern float act7Current;
    extern uint8_t act7State;
    extern Task *stop7;
    const uint8_t act7Pin1 = HAL::hBrg7Pin1;
    const uint8_t act7Pin2 = HAL::hBrg7Pin2;

    // TODO: Set correct OC limits
    const uint8_t OClimits[7] = {4,4,4,4,4,4,4}; //(amps): Act 1, Act 2, Act 3, Act 4, Act 5, Act 6, Act 7
    const float stopCurrent = .1; // Stopped actuator current threshold (amps) 
    
    void extendFuelFillRBV();
    void retractFuelFillRBV();
    uint32_t stopFuelFillRBV();
    void brakeFuelFillRBV();
    void fuelFillRBVPacketHandler(Comms::Packet tmp);

    void extendLoxFillRBV();
    void retractLoxFillRBV();
    uint32_t stopLoxFillRBV();
    void brakeLoxFillRBV();
    void loxFillRBVPacketHandler(Comms::Packet tmp);

    void extendPressFillRBV();
    void retractPressFillRBV();
    uint32_t stopPressFillRBV();
    void brakePressFillRBV();
    void pressFillRBVPacketHandler(Comms::Packet tmp);

    void extendPressLineVentRBV();
    void retractPressLineVentRBV();
    uint32_t stopPressLineVentRBV();
    void brakePressLineVentRBV();
    void pressLineVentRBVPacketHandler(Comms::Packet tmp);

    void extendAct5();
    void retractAct5();
    uint32_t stopAct5();
    void brakeAct5();
    void act5PacketHandler(Comms::Packet tmp);

    void extendAct6();
    void retractAct6();
    uint32_t stopAct6();
    void brakeAct6();
    void act6PacketHandler(Comms::Packet tmp);

    void extendAct7();
    void retractAct7();
    uint32_t stopAct7();
    void brakeAct7();
    void act7PacketHandler(Comms::Packet tmp);

    void actPacketHandler(Comms::Packet tmp, void (*extend)(), void (*retract)(), Task *stopTask);

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current, uint8_t *actState, uint8_t actuatorID);

    void initActuators();

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current);
    uint32_t fuelFillRBVSample();
    uint32_t loxFillRBVSample();
    uint32_t pressFillRBVSample();
    uint32_t pressLineVentRBVSample();
    uint32_t act5Sample();
    uint32_t act6Sample();
    uint32_t act7Sample();
};