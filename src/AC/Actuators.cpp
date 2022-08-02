#include "Actuators.h"

namespace Actuators {

    // TODO: change this to appropriate value
    uint32_t actuatorCheckPeriod = 50 * 1000;

    // TODO: set correct telem packet IDs
    /*
    Propane tank vent - 1
    LOx tank vent - 2
    Propane flow - 3
    LOx flow - 4
    Press fill - 5
    Press line vent - 6
    Igniter - 7
    */

   // act1
    Comms::Packet propTankVentRBVPacket = {.id = 70};
    uint8_t propTankVentRBVState = 0;
    float propTankVentRBVVoltage = 0.0;
    float propTankVentRBVCurrent = 0.0;
    Task *stop1;
    
    // act2
    Comms::Packet loxTankVentRBVPacket = {.id = 71};
    uint8_t loxTankVentRBVState = 0;
    float loxTankVentRBVVoltage = 0.0;
    float loxTankVentRBVCurrent = 0.0;
    Task *stop2;

    // act3
    Comms::Packet propFlowRBVPacket = {.id = 72};
    uint8_t propFlowRBVState = 0;
    float propFlowRBVVoltage = 0.0;
    float propFlowCurrent = 0.0;
    Task *stop3;

    // act4
    Comms::Packet loxFlowRBVPacket = {.id = 73};
    uint8_t loxFlowRBVState = 0;
    float loxFlowRBVVoltage = 0.0;
    float loxFlowRBVCurrent = 0.0;
    Task *stop4;

    // act5
    Comms::Packet pressFillRBVPacket = {.id = 74};
    uint8_t pressFillRBVState = 0;
    float pressFillRBVVoltage = 0.0;
    float pressFillRBVCurrent = 0.0;
    Task *stop5;

    // act6
    Comms::Packet pressLineVentRBVPacket = {.id = 75};
    uint8_t pressLineVentRBVState = 0;
    float pressLineVentRBVVoltage = 0.0;
    float pressLineVentRBVCurrent = 0.0;
    Task *stop6;

    // act7
    // TODO: Set to Igniter for now, but we can put another thing on this channel -- 
    // Igniter just needs 1 pin for on/off
    Comms::Packet igniterPacket = {.id = 76};
    uint8_t igniterState = 0;
    float igniterVoltage = 0.0;
    float igniterCurrent = 0.0;
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

    void extendPropTankVentRBV(){ driveForwards(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); }
    void retractPropTankVentRBV(){ driveBackwards(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); }
    uint32_t stopPropTankVentRBV(){ stopAct(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); stop1->enabled = false; return 0;}
    void brakePropTankVentRBV(){ brakeAct(propTankVentRBVPin1, propTankVentRBVPin2, &propTankVentRBVState, 0); }
    void propTankVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendpropTankVentRBV, &retractpropTankVentRBV, stop1); }

    void extendLoxTankVentRBV(){ driveForwards(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
    void retractLoxTankVentRBV(){ driveBackwards(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
    uint32_t stopLoxTankVentRBV(){ stopAct(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); stop2->enabled = false; return 0;}
    void brakeLoxTankVentRBV(){ brakeAct(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
    void loxTankVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendloxTankVentRBV, &retractloxTankVentRBV, stop2); }

    void extendPropFlowRBV(){ driveForwards(propFlowRBVPin1, propFlowRBVPin2, &propFlowRBVState, 2); }
    void retractPropFlowRBV(){ driveBackwards(propFlowRBVPin1, propFlowRBVPin2, &propFlowRBVState, 2); }
    uint32_t stopPropFlowRBV(){ stopAct(propFlowRBVPin1, propFlowRBVPin2, &propFlowRBVState, 2); stop3->enabled = false; return 0;}
    void brakePropFlowRBV(){ brakeAct(propFlowRBVPin1, propFlowRBVPin2, &propFlowRBVState, 2); }
    void propFlowRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendpropFlowRBV, &retractpropFlowRBV, stop3); }

    void extendLoxFlowRBV(){ driveForwards(loxFlowRBVPin1, loxFlowRBVPin2, &loxFlowRBVState, 3); }
    void retractLoxFlowRBV(){ driveBackwards(loxFlowRBVPin1, loxFlowRBVPin2, &loxFlowRBVState, 3); }
    uint32_t stopLoxFlowRBV(){ stopAct(loxFlowRBVPin1, loxFlowRBVPin2, &loxFlowRBVState, 3); stop4->enabled = false; return 0;}
    void brakeLoxFlowRBV(){ brakeAct(loxFlowRBVPin1, loxFlowRBVPin2, &loxFlowRBVState, 3); }
    void loxFlowRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendloxFlowRBV, &retractloxFlowRBV, stop4); }

    void extendPressFillRBV(){ driveForwards(pressFillRBVPin1, pressFillRBVPin2, &pressFillRBVState, 4); }
    void retractPressFillRBV(){ driveBackwards(pressFillRBVPin1, pressFillRBVPin2, &pressFillRBVState, 4); }
    uint32_t stopPressFillRBV(){ stopAct(pressFillRBVPin1, pressFillRBVPin2, &pressFillRBVState, 4); stop5->enabled = false; return 0;}
    void brakePressFillRBV(){ brakeAct(pressFillRBVPin1, pressFillRBVPin2, &pressFillRBVState, 4); }
    void pressFillRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendpressFillRBV, &retractpressFillRBV, stop5); }

    void extendPressLineVentRBV(){ driveForwards(pressLineVentRBVPin1, pressLineVentRBVPin2, &pressLineVentRBVState, 5); }
    void retractPressLineVentRBV(){ driveBackwards(pressLineVentRBVPin1, pressLineVentRBVPin2, &pressLineVentRBVState, 5); }
    uint32_t stopPressLineVentRBV(){ stopAct(pressLineVentRBVPin1, pressLineVentRBVPin2, &pressLineVentRBVState, 5); stop6->enabled = false; return 0;}
    void brakePressLineVentRBV(){ brakeAct(pressLineVentRBVPin1, pressLineVentRBVPin2, &pressLineVentRBVState, 5); }
    void pressLineVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendpressLineVentRBV, &retractpressLineVentRBV, stop6); }

    void extendIgniter(){ driveForwards(igniterPin1, igniterPin2, &igniterState, 6); }
    void retractIgniter(){ driveBackwards(igniterPin1, igniterPin2, &igniterState, 6); }
    uint32_t stopIgniter(){ stopAct(igniterPin1, igniterPin2, &igniterState, 6); stop7->enabled = false; return 0;}
    void brakeIgniter(){ brakeAct(igniterPin1, igniterPin2, &igniterState, 6); }
    void igniterPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendigniter, &retractigniter, stop7); }

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
                case 0: stoppropTankVentRBV(); break;
                case 1: stoploxTankVentRBV(); break;
                case 2: stoppropFlowRBV(); break;
                case 3: stoploxFlowRBV(); break;
                case 4: stoppressFillRBV(); break;
                case 5: stoppressLineVentRBV(); break;
                case 6: stopigniter(); break;
            }
            *actState = 3;
        }

        if ((*actState == 1 || *actState == 2) && *current < stopCurrent){
            switch(actuatorID){
                case 0: stoppropTankVentRBV(); break;
                case 1: stoploxTankVentRBV(); break;
                case 2: stoppropFlowRBV(); break;
                case 3: stoploxFlowRBV(); break;
                case 4: stoppressFillRBV(); break;
                case 5: stoppressLineVentRBV(); break;
                case 6: stopigniter(); break;
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

    uint32_t propFlowRBVSample() {
        sampleActuator(&propFlowRBVPacket, &HAL::chan6, &propFlowRBVVoltage, &propFlowRBVCurrent, &propFlowRBVState, 2);
        return actuatorCheckPeriod;
    }

    uint32_t loxFlowRBVSample() {
        sampleActuator(&loxFlowRBVPacket, &HAL::chan7, &loxFlowRBVVoltage, &loxFlowRBVCurrent, &loxFlowRBVState, 3);
        return actuatorCheckPeriod;
    }

    uint32_t pressFillRBVSample() {
        sampleActuator(&pressFillRBVPacket, &HAL::chan8, &pressFillRBVVoltage, &pressFillRBVCurrent, &pressFillRBVState, 4);
        return actuatorCheckPeriod;
    }

    uint32_t pressLineVentRBVSample() {
        sampleActuator(&pressLineVentRBVPacket, &HAL::chan9, &pressLineVentRBVVoltage, &pressLineVentRBVCurrent, &pressLineVentRBVState, 5);
        return actuatorCheckPeriod;
    }

    uint32_t igniterSample() {
        sampleActuator(&igniterPacket, &HAL::chan10, &igniterVoltage, &igniterCurrent, &igniterState, 6);
        return actuatorCheckPeriod;
    }

    void initActuators() {
        Comms::registerCallback(170, propTankVentRBVPacketHandler);
        Comms::registerCallback(171, loxTankVentRBVPacketHandler);
        Comms::registerCallback(172, propFlowRBVPacketHandler);
        Comms::registerCallback(173, loxFlowRBVPacketHandler);
        Comms::registerCallback(174, pressFillRBVPacketHandler);
        Comms::registerCallback(175, pressLineVentRBVPacketHandler);
        Comms::registerCallback(176, igniterPacketHandler);
    }
};