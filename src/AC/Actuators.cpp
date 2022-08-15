#include "Actuators.h"

namespace Actuators {

    // TODO: change this to appropriate value
    uint32_t actuatorCheckPeriod = 50 * 1000;

    // TODO: set correct telem packet IDs
    /*
    H Bridge
    Propane tank vent - 1
    LOx tank vent - 2
    Propane fill - 3
    LOx fill - 4
    */

   // act1
    Comms::Packet propTankVentRBVPacket = {.id = 70};
    uint8_t propTankVentRBVState = 0;
    float propTankVentRBVVoltage = 0.0;
    float propTankVentRBVCurrent = 0.0;
    Task *stopPropTankVentRBVTask;
    
    // act2
    Comms::Packet loxTankVentRBVPacket = {.id = 71};
    uint8_t loxTankVentRBVState = 0;
    float loxTankVentRBVVoltage = 0.0;
    float loxTankVentRBVCurrent = 0.0;
    Task *stopLoxTankVentRBVTask;

    // act3
    Comms::Packet propFillRBVPacket = {.id = 72};
    uint8_t propFillRBVState = 0;
    float propFillRBVVoltage = 0.0;
    float propFillRBVCurrent = 0.0;
    Task *stopPropFillRBVTask;

    // act4
    Comms::Packet loxFillRBVPacket = {.id = 73};
    uint8_t loxFillRBVState = 0;
    float loxFillRBVVoltage = 0.0;
    float loxFillRBVCurrent = 0.0;
    Task *stopLoxFillRBVTask;

    // act5
    Comms::Packet twoWayPacket = {.id = 74};
    uint8_t twoWayState = 0;
    float twoWayVoltage = 0.0;
    float twoWayCurrent = 0.0;
    Task *stopTwoWayTask;

    // act6
    Comms::Packet loxGemsPacket = {.id = 75 };
    uint8_t loxGemsState = 0;
    float loxGemsVoltage = 0.0;
    float loxGemsCurrent = 0.0;
    Task *stopLoxGemsTask;

    // act7
    Comms::Packet igniterPacket = {.id = 76 };
    uint8_t igniterState = 0;
    float igniterVoltage = 0.0;
    float igniterCurrent = 0.0;
    Task *stopIgniterTask;

    void driveForwards(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
        digitalWriteFast(pin1, HIGH);
        digitalWriteFast(pin2, LOW);
        *actState = 1;
    }

    void driveBackwards(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
        digitalWriteFast(pin1, LOW);
        digitalWriteFast(pin2, HIGH);
        *actState = 2;
    }

    void stopAct(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
        digitalWriteFast(pin1, LOW);
        digitalWriteFast(pin2, LOW);
        *actState = 0;
    }

    void brakeAct(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
        digitalWriteFast(pin1, HIGH);
        digitalWriteFast(pin2, HIGH);
        *actState = 4; // Probably won't brake
    }

    void extendPropTankVentRBV(){ driveForwards(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); }
    void retractPropTankVentRBV(){ driveBackwards(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); }
    uint32_t stopPropTankVentRBV(){ stopAct(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); stopPropTankVentRBVTask->enabled = false; return 0;}
    void brakePropTankVentRBV(){ brakeAct(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); }
    void propTankVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendPropTankVentRBV, &retractPropTankVentRBV, stopPropTankVentRBVTask); }

    void extendLoxTankVentRBV(){ driveForwards(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
    void retractLoxTankVentRBV(){ driveBackwards(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
    uint32_t stopLoxTankVentRBV(){ stopAct(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); stopLoxTankVentRBVTask->enabled = false; return 0;}
    void brakeLoxTankVentRBV(){ brakeAct(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
    void loxTankVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendLoxFillRBV, &retractLoxFillRBV, stopLoxTankVentRBVTask); }

    void extendPropFillRBV(){ driveForwards(propFillRBVPin1, propFillRBVPin2, &propFillRBVState, 2); }
    void retractPropFillRBV(){ driveBackwards(propFillRBVPin1, propFillRBVPin2, &propFillRBVState, 2); }
    uint32_t stopPropFillRBV(){ stopAct(propFillRBVPin1, propFillRBVPin2, &propFillRBVState, 2); stopPropFillRBVTask->enabled = false; return 0;}
    void brakePropFillRBV(){ brakeAct(propFillRBVPin1, propFillRBVPin2, &propFillRBVState, 2); }
    void propFillRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendPropFillRBV, &retractPropFillRBV, stopPropFillRBVTask); }

    void extendLoxFillRBV(){ driveForwards(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 3); }
    void retractLoxFillRBV(){ driveBackwards(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 3); }
    uint32_t stopLoxFillRBV(){ stopAct(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 3); stopLoxFillRBVTask->enabled = false; return 0;}
    void brakeLoxFillRBV(){ brakeAct(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 3); }
    void loxFillRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendLoxFillRBV, &retractLoxFillRBV, stopLoxFillRBVTask); }

    void extendTwoWay(){ driveForwards(twoWayPin1, twoWayPin2, &twoWayState, 3); }
    void retractTwoWay(){ driveBackwards(twoWayPin1, twoWayPin2, &twoWayState, 3); }
    uint32_t stopTwoWay(){ stopAct(twoWayPin1, twoWayPin2, &twoWayState, 3); stopTwoWayTask->enabled = false; return 0;}
    void brakeTwoWay(){ brakeAct(twoWayPin1, twoWayPin2, &twoWayState, 3); }
    void twoWayPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendTwoWay, &retractTwoWay, stopTwoWayTask); }

    void extendLoxGems(){ driveForwards(loxGemsPin1, loxGemsPin2, &loxGemsState, 3); }
    void retractLoxGems(){ driveBackwards(loxGemsPin1, loxGemsPin2, &loxGemsState, 3); }
    uint32_t stopLoxGems(){ stopAct(loxGemsPin1, loxGemsPin2, &loxGemsState, 3); stopLoxGemsTask->enabled = false; return 0;}
    void brakeLoxGems(){ brakeAct(loxGemsPin1, loxGemsPin2, &loxGemsState, 3); }
    void loxGemsPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendLoxGems, &retractLoxGems, stopLoxGemsTask); }

    void extendIgniter(){ driveForwards(igniterPin1, igniterPin2, &igniterState, 3); }
    void retractIgniter(){ driveBackwards(igniterPin1, igniterPin2, &igniterState, 3); }
    uint32_t stopIgniter(){ stopAct(igniterPin1, igniterPin2, &igniterState, 3); stopIgniterTask->enabled = false; return 0;}
    void brakeIgniter(){ brakeAct(igniterPin1, igniterPin2, &igniterState, 3); }
    void igniterPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendIgniter, &retractIgniter, stopIgniterTask); }
    
    void actPacketHandler(Comms::Packet tmp, void (*extend)(), void (*retract)(), Task *stopTask){
        if(tmp.data[0]%2)(*extend)();
        else (*retract)();

        if(tmp.data[0]>1){
            uint32_t actuatetime = Comms::packetGetUint32(&tmp, 1);
            if(stopTask->enabled) stopTask->nexttime += actuatetime * 1000;
            else stopTask->nexttime = micros() + actuatetime * 1000;
            stopTask->enabled = true;
        }
    }

    void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current, uint8_t *actState, uint8_t actuatorID) {
        *voltage = ina->readBusVoltage();
        *current = ina->readShuntCurrent();

        if (*current > OClimits[actuatorID]){
            switch(actuatorID){
                case 0: stopPropTankVentRBV(); break;
                case 1: stopLoxTankVentRBV(); break;
                case 2: stopPropFillRBV(); break;
                case 3: stopLoxFillRBV(); break;
                case 4: stopTwoWay(); break;
                case 5: stopLoxGems(); break;
                case 6: stopIgniter(); break;
            }
            *actState = 3;
        }

        if ((*actState == 1 || *actState == 2) && *current < stopCurrent){
            switch(actuatorID){
                case 0: stopPropTankVentRBV(); break;
                case 1: stopLoxTankVentRBV(); break;
                case 2: stopPropFillRBV(); break;
                case 3: stopLoxFillRBV(); break;
                case 4: stopTwoWay(); break;
                case 5: stopLoxGems(); break;
                case 6: stopIgniter(); break;
            }
        }

        packet->len = 0;
        Comms::packetAddUint8(packet, *actState);
        Comms::packetAddFloat(packet, *voltage);
        Comms::packetAddFloat(packet, *current);
        Comms::emitPacket(packet);
    }

    uint32_t propTankVentRBVSample() {
        sampleActuator(&propTankVentRBVPacket, &HAL::chan4, &propTankVentRBVVoltage, &propTankVentRBVCurrent, &propTankVentRBVState, 0);
        return actuatorCheckPeriod;
    }

    uint32_t loxTankVentRBVSample() {
        sampleActuator(&loxTankVentRBVPacket, &HAL::chan5, &loxTankVentRBVVoltage, &loxTankVentRBVCurrent, &loxTankVentRBVState, 1);
        return actuatorCheckPeriod;
    }

    uint32_t propFillRBVSample() {
        sampleActuator(&propFillRBVPacket, &HAL::chan6, &propFillRBVVoltage, &propFillRBVCurrent, &propFillRBVState, 2);
        return actuatorCheckPeriod;
    }

    uint32_t loxFillRBVSample() {
        sampleActuator(&loxFillRBVPacket, &HAL::chan7, &loxFillRBVVoltage, &loxFillRBVCurrent, &loxFillRBVState, 3);
        return actuatorCheckPeriod;
    }

    uint32_t twoWaySample() {
        sampleActuator(&twoWayPacket, &HAL::chan7, &twoWayVoltage, &twoWayCurrent, &twoWayState, 3);
        return actuatorCheckPeriod;
    }

    uint32_t loxGemsSample() {
        sampleActuator(&loxGemsPacket, &HAL::chan7, &loxGemsVoltage, &loxGemsCurrent, &loxGemsState, 3);
        return actuatorCheckPeriod;
    }

    uint32_t igniterSample() {
        sampleActuator(&igniterPacket, &HAL::chan7, &igniterVoltage, &igniterCurrent, &igniterState, 3);
        return actuatorCheckPeriod;
    }

    void initActuators() {
        Comms::registerCallback(10, propTankVentRBVPacketHandler);
        Comms::registerCallback(11, loxTankVentRBVPacketHandler);
        Comms::registerCallback(12, propFillRBVPacketHandler);
        Comms::registerCallback(13, loxFillRBVPacketHandler);
        //TODO add other channels
    }
};