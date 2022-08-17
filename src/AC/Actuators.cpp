// #include "Actuators.h"

// namespace Actuators {

//     // TODO: change this to appropriate value
//     uint32_t actuatorCheckPeriod = 50 * 1000;

//     // TODO: set correct telem packet IDs
//     /*
//     H Bridge
//     Propane tank vent - 1
//     LOx tank vent - 2
//     Propane fill - 3
//     LOx fill - 4
//     */

//    // act1
//     Comms::Packet propTankVentRBVPacket = {.id = 70};
//     uint8_t propTankVentRBVState = 0;
//     float propTankVentRBVVoltage = 0.0;
//     float propTankVentRBVCurrent = 0.0;
//     Task *stopPropTankVentRBVTask;
    
//     // act2
//     Comms::Packet loxTankVentRBVPacket = {.id = 71};
//     uint8_t loxTankVentRBVState = 0;
//     float loxTankVentRBVVoltage = 0.0;
//     float loxTankVentRBVCurrent = 0.0;
//     Task *stopLoxTankVentRBVTask;

//     // act3
//     Comms::Packet propFillRBVPacket = {.id = 72};
//     uint8_t propFillRBVState = 0;
//     float propFillRBVVoltage = 0.0;
//     float propFillRBVCurrent = 0.0;
//     Task *stopPropFillRBVTask;

//     // act4
//     Comms::Packet loxFillRBVPacket = {.id = 73};
//     uint8_t loxFillRBVState = 0;
//     float loxFillRBVVoltage = 0.0;
//     float loxFillRBVCurrent = 0.0;
//     Task *stopLoxFillRBVTask;

//     // act5
//     Comms::Packet twoWayPacket = {.id = 74};
//     uint8_t twoWayState = 0;
//     float twoWayVoltage = 0.0;
//     float twoWayCurrent = 0.0;
//     Task *stopTwoWayTask;

//     // act6
//     Comms::Packet loxGemsPacket = {.id = 75 };
//     uint8_t loxGemsState = 0;
//     float loxGemsVoltage = 0.0;
//     float loxGemsCurrent = 0.0;
//     Task *stopLoxGemsTask;

//     // act7
//     Comms::Packet igniterPacket = {.id = 76 };
//     uint8_t igniterState = 0;
//     float igniterVoltage = 0.0;
//     float igniterCurrent = 0.0;
//     Task *stopIgniterTask;

//     void driveForwards(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
//         digitalWriteFast(pin1, HIGH);
//         digitalWriteFast(pin2, LOW);
//         *actState = 1;
//     }

//     void driveBackwards(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
//         digitalWriteFast(pin1, LOW);
//         digitalWriteFast(pin2, HIGH);
//         *actState = 2;
//     }

//     void stopAct(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
//         digitalWriteFast(pin1, LOW);
//         digitalWriteFast(pin2, LOW);
//         *actState = 0;
//     }

//     void brakeAct(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
//         digitalWriteFast(pin1, HIGH);
//         digitalWriteFast(pin2, HIGH);
//         *actState = 4; // Probably won't brake
//     }

//     void extendPropTankVentRBV(){ driveForwards(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); }
//     void retractPropTankVentRBV(){ driveBackwards(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); }
//     uint32_t stopPropTankVentRBV(){ stopAct(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); stopPropTankVentRBVTask->enabled = false; return 0;}
//     void brakePropTankVentRBV(){ brakeAct(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); }
//     void propTankVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendPropTankVentRBV, &retractPropTankVentRBV, stopPropTankVentRBVTask); }

//     void extendLoxTankVentRBV(){ driveForwards(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
//     void retractLoxTankVentRBV(){ driveBackwards(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
//     uint32_t stopLoxTankVentRBV(){ stopAct(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); stopLoxTankVentRBVTask->enabled = false; return 0;}
//     void brakeLoxTankVentRBV(){ brakeAct(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
//     void loxTankVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendLoxTankVentRBV, &retractLoxTankVentRBV, stopLoxTankVentRBVTask); }

//     void extendPropFillRBV(){ driveForwards(propFillRBVPin1, propFillRBVPin2, &propFillRBVState, 2); }
//     void retractPropFillRBV(){ driveBackwards(propFillRBVPin1, propFillRBVPin2, &propFillRBVState, 2); }
//     uint32_t stopPropFillRBV(){ stopAct(propFillRBVPin1, propFillRBVPin2, &propFillRBVState, 2); stopPropFillRBVTask->enabled = false; return 0;}
//     void brakePropFillRBV(){ brakeAct(propFillRBVPin1, propFillRBVPin2, &propFillRBVState, 2); }
//     void propFillRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendPropFillRBV, &retractPropFillRBV, stopPropFillRBVTask); }

//     void extendLoxFillRBV(){ driveForwards(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 3); }
//     void retractLoxFillRBV(){ driveBackwards(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 3); }
//     uint32_t stopLoxFillRBV(){ stopAct(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 3); stopLoxFillRBVTask->enabled = false; return 0;}
//     void brakeLoxFillRBV(){ brakeAct(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 3); }
//     void loxFillRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendLoxFillRBV, &retractLoxFillRBV, stopLoxFillRBVTask); }

//     void extendTwoWay(){ driveForwards(twoWayPin1, twoWayPin2, &twoWayState, 4); }
//     void retractTwoWay(){ driveBackwards(twoWayPin1, twoWayPin2, &twoWayState, 4); }
//     uint32_t stopTwoWay(){ stopAct(twoWayPin1, twoWayPin2, &twoWayState, 4); stopTwoWayTask->enabled = false; return 0;}
//     void brakeTwoWay(){ brakeAct(twoWayPin1, twoWayPin2, &twoWayState, 4); }
//     void twoWayPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendTwoWay, &retractTwoWay, stopTwoWayTask); }

//     void extendLoxGems(){ driveForwards(loxGemsPin1, loxGemsPin2, &loxGemsState, 5); }
//     void retractLoxGems(){ driveBackwards(loxGemsPin1, loxGemsPin2, &loxGemsState, 5); }
//     uint32_t stopLoxGems(){ stopAct(loxGemsPin1, loxGemsPin2, &loxGemsState, 5); stopLoxGemsTask->enabled = false; return 0;}
//     void brakeLoxGems(){ brakeAct(loxGemsPin1, loxGemsPin2, &loxGemsState, 5); }
//     void loxGemsPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendLoxGems, &retractLoxGems, stopLoxGemsTask); }

//     void extendIgniter(){ driveForwards(igniterPin1, igniterPin2, &igniterState, 6); }
//     void retractIgniter(){ driveBackwards(igniterPin1, igniterPin2, &igniterState, 6); }
//     uint32_t stopIgniter(){ stopAct(igniterPin1, igniterPin2, &igniterState, 6); stopIgniterTask->enabled = false; return 0;}
//     void brakeIgniter(){ brakeAct(igniterPin1, igniterPin2, &igniterState, 6); }
//     void igniterPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendIgniter, &retractIgniter, stopIgniterTask); }
    
//     void actPacketHandler(Comms::Packet tmp, void (*extend)(), void (*retract)(), Task *stopTask){
//         if(tmp.data[0]%2)(*extend)();
//         else (*retract)();

//         if(tmp.data[0]>1){
//             uint32_t actuatetime = Comms::packetGetUint32(&tmp, 1);
//             if(stopTask->enabled) stopTask->nexttime += actuatetime * 1000;
//             else stopTask->nexttime = micros() + actuatetime * 1000;
//             stopTask->enabled = true;
//         }
//     }

//     void sampleActuator(Comms::Packet *packet, INA219 *ina, float *voltage, float *current, uint8_t *actState, uint8_t actuatorID) {
//         *voltage = ina->readBusVoltage();
//         *current = ina->readShuntCurrent();
//         DEBUGF("Actuator %d. Current %f, state %d\n", actuatorID, *current, *actState);

//         if (*current > OClimits[actuatorID]){
//             switch(actuatorID){
//                 case 0: stopPropTankVentRBV(); break;
//                 case 1: stopLoxTankVentRBV(); break;
//                 case 2: stopPropFillRBV(); break;
//                 case 3: stopLoxFillRBV(); break;
//                 case 4: stopTwoWay(); break;
//                 case 5: stopLoxGems(); break;
//                 case 6: stopIgniter(); break;
//             }
//             *actState = 3;
//         }

//         if ((*actState == 1 || *actState == 2) && *current < stopCurrent){
//             switch(actuatorID){
//                 case 0: stopPropTankVentRBV(); break;
//                 case 1: stopLoxTankVentRBV(); break;
//                 case 2: stopPropFillRBV(); break;
//                 case 3: stopLoxFillRBV(); break;
//                 case 4: stopTwoWay(); break;
//                 case 5: stopLoxGems(); break;
//                 case 6: stopIgniter(); break;
//             }
//         }

//         packet->len = 0;
//         Comms::packetAddUint8(packet, *actState);
//         Comms::packetAddFloat(packet, *voltage);
//         Comms::packetAddFloat(packet, *current);
//         Comms::emitPacket(packet);
//     }

//     uint32_t propTankVentRBVSample() {
//         sampleActuator(&propTankVentRBVPacket, &HAL::chan4, &propTankVentRBVVoltage, &propTankVentRBVCurrent, &propTankVentRBVState, 0);
//         return actuatorCheckPeriod;
//     }

//     uint32_t loxTankVentRBVSample() {
//         sampleActuator(&loxTankVentRBVPacket, &HAL::chan5, &loxTankVentRBVVoltage, &loxTankVentRBVCurrent, &loxTankVentRBVState, 1);
//         return actuatorCheckPeriod;
//     }

//     uint32_t propFillRBVSample() {
//         sampleActuator(&propFillRBVPacket, &HAL::chan6, &propFillRBVVoltage, &propFillRBVCurrent, &propFillRBVState, 2);
//         return actuatorCheckPeriod;
//     }

//     uint32_t loxFillRBVSample() {
//         sampleActuator(&loxFillRBVPacket, &HAL::chan7, &loxFillRBVVoltage, &loxFillRBVCurrent, &loxFillRBVState, 3);
//         return actuatorCheckPeriod;
//     }

//     uint32_t twoWaySample() {
//         sampleActuator(&twoWayPacket, &HAL::chan8, &twoWayVoltage, &twoWayCurrent, &twoWayState, 4);
//         return actuatorCheckPeriod;
//     }

//     uint32_t loxGemsSample() {
//         sampleActuator(&loxGemsPacket, &HAL::chan9, &loxGemsVoltage, &loxGemsCurrent, &loxGemsState, 5);
//         return actuatorCheckPeriod;
//     }

//     uint32_t igniterSample() {
//         sampleActuator(&igniterPacket, &HAL::chan10, &igniterVoltage, &igniterCurrent, &igniterState, 6);
//         return actuatorCheckPeriod;
//     }

//     void initActuators() {
//         Comms::registerCallback(10, propTankVentRBVPacketHandler);
//         Comms::registerCallback(11, loxTankVentRBVPacketHandler);
//         Comms::registerCallback(12, propFillRBVPacketHandler);
//         Comms::registerCallback(13, loxFillRBVPacketHandler);
//         Comms::registerCallback(14, twoWayPacketHandler);
//         Comms::registerCallback(15, loxGemsPacketHandler);
//         Comms::registerCallback(16, igniterPacketHandler);

//     }
// };



#include "Actuators.h"

namespace Actuators {

    // TODO: change this to appropriate value
    uint32_t actuatorCheckPeriod = 50 * 1000;

    // TODO: set correct telem packet IDs
    Comms::Packet act1Packet = {.id = 70};
    uint8_t act1State = 0;
    float act1Voltage = 0.0;
    float act1Current = 0.0;
    Task *stop1;
    
    Comms::Packet act2Packet = {.id = 71};
    uint8_t act2State = 0;
    float act2Voltage = 0.0;
    float act2Current = 0.0;
    Task *stop2;

    Comms::Packet act3Packet = {.id = 72};
    uint8_t act3State = 0;
    float act3Voltage = 0.0;
    float act3Current = 0.0;
    Task *stop3;

    Comms::Packet act4Packet = {.id = 73};
    uint8_t act4State = 0;
    float act4Voltage = 0.0;
    float act4Current = 0.0;
    Task *stop4;

    Comms::Packet act5Packet = {.id = 74};
    uint8_t act5State = 0;
    float act5Voltage = 0.0;
    float act5Current = 0.0;
    Task *stop5;

    Comms::Packet act6Packet = {.id = 75};
    uint8_t act6State = 0;
    float act6Voltage = 0.0;
    float act6Current = 0.0;
    Task *stop6;

    Comms::Packet act7Packet = {.id = 76};
    uint8_t act7State = 0;
    float act7Voltage = 0.0;
    float act7Current = 0.0;
    Task *stop7;

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

    void extendAct1(){ driveForwards(act1Pin1, act1Pin2, &act1State, 0); }
    void retractAct1(){ driveBackwards(act1Pin1, act1Pin2, &act1State, 0); }
    uint32_t stopAct1(){ stopAct(act1Pin1, act1Pin2, &act1State, 0); stop1->enabled = false; return 0;}
    void brakeAct1(){ brakeAct(act1Pin1, act1Pin2, &act1State, 0); }
    void act1PacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendAct1, &retractAct1, stop1); }

    void extendAct2(){ driveForwards(act2Pin1, act2Pin2, &act2State, 1); }
    void retractAct2(){ driveBackwards(act2Pin1, act2Pin2, &act2State, 1); }
    uint32_t stopAct2(){ stopAct(act2Pin1, act2Pin2, &act2State, 1); stop2->enabled = false; return 0;}
    void brakeAct2(){ brakeAct(act2Pin1, act2Pin2, &act2State, 1); }
    void act2PacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendAct2, &retractAct2, stop2); }

    void extendAct3(){ driveForwards(act3Pin1, act3Pin2, &act3State, 2); }
    void retractAct3(){ driveBackwards(act3Pin1, act3Pin2, &act3State, 2); }
    uint32_t stopAct3(){ stopAct(act3Pin1, act3Pin2, &act3State, 2); stop3->enabled = false; return 0;}
    void brakeAct3(){ brakeAct(act3Pin1, act3Pin2, &act3State, 2); }
    void act3PacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendAct3, &retractAct3, stop3); }

    void extendAct4(){ driveForwards(act4Pin1, act4Pin2, &act4State, 3); }
    void retractAct4(){ driveBackwards(act4Pin1, act4Pin2, &act4State, 3); }
    uint32_t stopAct4(){ stopAct(act4Pin1, act4Pin2, &act4State, 3); stop4->enabled = false; return 0;}
    void brakeAct4(){ brakeAct(act4Pin1, act4Pin2, &act4State, 3); }
    void act4PacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendAct4, &retractAct4, stop4); }

    void extendAct5(){ driveForwards(act5Pin1, act5Pin2, &act5State, 4); }
    void retractAct5(){ driveBackwards(act5Pin1, act5Pin2, &act5State, 4); }
    uint32_t stopAct5(){ stopAct(act5Pin1, act5Pin2, &act5State, 4); stop5->enabled = false; return 0;}
    void brakeAct5(){ brakeAct(act5Pin1, act5Pin2, &act5State, 4); }
    void act5PacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendAct5, &retractAct5, stop5); }

    void extendAct6(){ driveForwards(act6Pin1, act6Pin2, &act6State, 5); }
    void retractAct6(){ driveBackwards(act6Pin1, act6Pin2, &act6State, 5); }
    uint32_t stopAct6(){ stopAct(act6Pin1, act6Pin2, &act6State, 5); stop6->enabled = false; return 0;}
    void brakeAct6(){ brakeAct(act6Pin1, act6Pin2, &act6State, 5); }
    void act6PacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendAct6, &retractAct6, stop6); }

    void extendAct7(){ driveForwards(act7Pin1, act7Pin2, &act7State, 6); }
    void retractAct7(){ driveBackwards(act7Pin1, act7Pin2, &act7State, 6); }
    uint32_t stopAct7(){ stopAct(act7Pin1, act7Pin2, &act7State, 6); stop7->enabled = false; return 0;}
    void brakeAct7(){ brakeAct(act7Pin1, act7Pin2, &act7State, 6); }
    void act7PacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendAct7, &retractAct7, stop7); }

    void actPacketHandler(Comms::Packet tmp, void (*extend)(), void (*retract)(), Task *stopTask){
/*         switch(tmp.data[0]){
            case 0:
                (*extend)();
                break;
            case 1:
                (*retract)();
                break;
            case 2:
                (*extend)();
                if(stopTask->enabled) stopTask->nexttime += Comms::packetGetUint32(&tmp, 1) * 1000;
                else stopTask->nexttime = micros() + Comms::packetGetUint32(&tmp, 1) * 1000;
                stopTask->enabled = true;
                break;
            case 3:
                (*retract)();
                if(stopTask->enabled) stopTask->nexttime += Comms::packetGetUint32(&tmp, 1) * 1000;
                else stopTask->nexttime = micros() + Comms::packetGetUint32(&tmp, 1) * 1000;
                stopTask->enabled = true;
                break;
        } */

        // (Actuate code) 0: extend fully 1: retract fully 2: extend millis 3: retract millis

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
                case 0: stopAct1(); break;
                case 1: stopAct2(); break;
                case 2: stopAct3(); break;
                case 3: stopAct4(); break;
                case 4: stopAct5(); break;
                case 5: stopAct6(); break;
                case 6: stopAct7(); break;
            }
            *actState = 3;
        }

        if ((*actState == 1 || *actState == 2) && *current < stopCurrent){
            switch(actuatorID){
                case 0: stopAct1(); break;
                case 1: stopAct2(); break;
                case 2: stopAct3(); break;
                case 3: stopAct4(); break;
                case 4: stopAct5(); break;
                case 5: stopAct6(); break;
                case 6: stopAct7(); break;
            }
        }

        packet->len = 0;
        Comms::packetAddUint8(packet, *actState);
        Comms::packetAddFloat(packet, *voltage);
        Comms::packetAddFloat(packet, *current);
        Comms::emitPacket(packet);
    }

    uint32_t act1Sample() {
        sampleActuator(&act1Packet, &HAL::chan4, &act1Voltage, &act1Current, &act1State, 0);
        return actuatorCheckPeriod;
    }

    uint32_t act2Sample() {
        sampleActuator(&act2Packet, &HAL::chan5, &act2Voltage, &act2Current, &act2State, 1);
        return actuatorCheckPeriod;
    }

    uint32_t act3Sample() {
        sampleActuator(&act3Packet, &HAL::chan6, &act3Voltage, &act3Current, &act3State, 2);
        return actuatorCheckPeriod;
    }

    uint32_t act4Sample() {
        sampleActuator(&act4Packet, &HAL::chan7, &act4Voltage, &act4Current, &act4State, 3);
        return actuatorCheckPeriod;
    }

    uint32_t act5Sample() {
        sampleActuator(&act5Packet, &HAL::chan8, &act5Voltage, &act5Current, &act5State, 4);
        return actuatorCheckPeriod;
    }

    uint32_t act6Sample() {
        sampleActuator(&act6Packet, &HAL::chan9, &act6Voltage, &act6Current, &act6State, 5);
        return actuatorCheckPeriod;
    }

    uint32_t act7Sample() {
        sampleActuator(&act7Packet, &HAL::chan10, &act7Voltage, &act7Current, &act7State, 6);
        return actuatorCheckPeriod;
    }

    void initActuators() {
        Comms::registerCallback(10, act1PacketHandler);
        Comms::registerCallback(11, act2PacketHandler);
        Comms::registerCallback(12, act3PacketHandler);
        Comms::registerCallback(13, act4PacketHandler);
        Comms::registerCallback(14, act5PacketHandler);
        Comms::registerCallback(15, act6PacketHandler);
        Comms::registerCallback(16, act7PacketHandler);
    }
};