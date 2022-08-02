#pragma once

#include <Common.h>

#include "HAL.h"
#include <Comms.h>  

#include <Arduino.h>
#include <INA219.h>

namespace Actuators {
    
    extern uint32_t heaterCheckPeriod; // interval for checking valve current and voltages

    const float maxValveCurrent = 1.0;

    // ac1
    extern float propTankVentRBVVoltage;
    extern float propTankVentRBVCurrent;
    extern uint8_t propTankVentRBVState;
    extern Task *stop1;
    const uint8_t propTankVentRBVPin1 = HAL::hBrg1Pin1;
    const uint8_t propTankVentRBVPin2 = HAL::hBrg1Pin2;

    // act2
    extern float loxTankVentRBVVoltage;
    extern float loxTankVentRBVCurrent;
    extern uint8_t loxTankVentRBVState;
    extern Task *stop2;
    const uint8_t loxTankVentRBVPin1 = HAL::hBrg2Pin1;
    const uint8_t loxTankVentRBVPin2 = HAL::hBrg2Pin2;

    // act3
    extern float propFlowRBVVoltage;
    extern float propFlowRBVCurrent;
    extern uint8_t propFlowRBVState;
    extern Task *stop3;
    const uint8_t propFlowRBVPin1 = HAL::hBrg3Pin1;
    const uint8_t propFlowRBVPin2 = HAL::hBrg3Pin2;

    // act4
    extern float loxFlowRBVVoltage;
    extern float loxFlowRBVCurrent;
    extern uint8_t loxFlowRBVState;
    extern Task *stop4;
    const uint8_t loxFlowRBVPin1 = HAL::hBrg4Pin1;
    const uint8_t loxFlowRBVPin2 = HAL::hBrg4Pin2;

    // act5
    extern float pressFillRBVVoltage;
    extern float pressFillRBVCurrent;
    extern uint8_t pressFillRBVState;
    extern Task *stop5;
    const uint8_t pressFillRBVPin1 = HAL::hBrg5Pin1;
    const uint8_t pressFillRBVPin2 = HAL::hBrg5Pin2;

    // act6
    extern float pressLineVentRBVVoltage;
    extern float pressLineVentRBVCurrent;
    extern uint8_t pressLineVentRBVState;
    extern Task *stop6;
    const uint8_t pressLineVentRBVPin1 = HAL::hBrg6Pin1;
    const uint8_t pressLineVentRBVPin2 = HAL::hBrg6Pin2;

    // TODO: Currently mapped to Igniter, but may put something else on this channel as well? 
    extern float act7Voltage;
    extern float act7Current;
    extern uint8_t act7State;
    extern Task *stop7;
    // TODO: The separate pins may get mapped to different things, so TBD on this channel
    const uint8_t act7Pin1 = HAL::hBrg7Pin1;
    const uint8_t act7Pin2 = HAL::hBrg7Pin2;

    // TODO: Set correct OC limits
    const uint8_t OClimits[7] = {4,4,4,4,4,4,4}; //(amps): Act 1, Act 2, Act 3, Act 4, Act 5, Act 6, Act 7
    const float stopCurrent = .1; // Stopped actuator current threshold (amps) 
    
    void extendpropTankVentRBV();
    void retractpropTankVentRBV();
    uint32_t stoppropTankVentRBV();
    void brakepropTankVentRBV();
    void propTankVentRBVPacketHandler(Comms::Packet tmp);

    void extendloxTankVentRBV();
    void retractloxTankVentRBV();
    uint32_t stoploxTankVentRBV();
    void brakeloxTankVentRBV();
    void loxTankVentRBVPacketHandler(Comms::Packet tmp);

    void extendpropFlowRBV();
    void retractpropFlowRBV();
    uint32_t stoppropFlowRBV();
    void brakepropFlowRBV();
    void propFlowRBVPacketHandler(Comms::Packet tmp);

    void extendloxFlowRBV();
    void retractloxFlowRBV();
    uint32_t stoploxFlowRBV();
    void brakeloxFlowRBV();
    void loxFlowRBVPacketHandler(Comms::Packet tmp);

    void extendpressFillRBV();
    void retractpressFillRBV();
    uint32_t stoppressFillRBV();
    void brakepressFillRBV();
    void pressFillRBVPacketHandler(Comms::Packet tmp);

    void extendpressLineVentRBV();
    void retractpressLineVentRBV();
    uint32_t stoppressLineVentRBV();
    void brakepressLineVentRBV();
    void pressLineVentRBVPacketHandler(Comms::Packet tmp);

    void extendAct7();
    void retractAct7();
    uint32_t stopAct7();
    void brakeAct7();
    void act7PacketHandler(Comms::Packet tmp);

    void actPacketHandler(Comms::Packet tmp, void (*extend)(), void (*retract)(), Task *stopTask);

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current, uint8_t *actState, uint8_t actuatorID);

    void initActuators();

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current);
    uint32_t propTankVentRBVSample();
    uint32_t loxTankVentRBVSample();
    uint32_t propFlowRBVSample();
    uint32_t loxFlowRBVSample();
    uint32_t pressFillRBVSample();
    uint32_t pressLineVentRBVSample();
    uint32_t act7Sample();
};