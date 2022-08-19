#include "Actuators.h"

namespace Actuators {

    // TODO: change this to appropriate value
    uint32_t actuatorCheckPeriod = 50 * 1000;

    // TODO: set correct telem packet IDs
    Comms::Packet fuelTankVentRBVPacket = {.id = 70};
    uint8_t fuelTankVentRBVState = 0;
    float fuelTankVentRBVVoltage = 0.0;
    float fuelTankVentRBVCurrent = 0.0;
    Task *stopFuelTankVentRBVTask;
    
    Comms::Packet loxTankVentRBVPacket = {.id = 71};
    uint8_t loxTankVentRBVState = 0;
    float loxTankVentRBVVoltage = 0.0;
    float loxTankVentRBVCurrent = 0.0;
    Task *stopLoxTankVentRBVTask;

    Comms::Packet igniterEnablePacket = {.id = 72};
    uint8_t igniterEnableState = 0;
    float igniterEnableVoltage = 0.0;
    float igniterEnableCurrent = 0.0;
    Task *stopIgniterEnableTask;

    Comms::Packet twoWayPacket = {.id = 73};
    uint8_t twoWayState = 0;
    float twoWayVoltage = 0.0;
    float twoWayCurrent = 0.0;
    Task *stopTwoWayTask;

    Comms::Packet act5Packet = {.id = 74};
    uint8_t act5State = 0;
    float act5Voltage = 0.0;
    float act5Current = 0.0;
    Task *stop5;

    Comms::Packet fuelGemsPacket = {.id = 75};
    uint8_t fuelGemsState = 0;
    float fuelGemsVoltage = 0.0;
    float fuelGemsCurrent = 0.0;
    Task *stopFuelGemsTask;

    Comms::Packet loxGemsPacket = {.id = 76};
    uint8_t loxGemsState = 0;
    float loxGemsVoltage = 0.0;
    float loxGemsCurrent = 0.0;
    Task *stopLoxGemsTask;

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

    void extendFuelTankVentRBV(){ driveForwards(fuelTankVentRBVPin1, fuelTankVentRBVPin2, &fuelTankVentRBVState, 0); }
    void retractFuelTankVentRBV(){ driveBackwards(fuelTankVentRBVPin1, fuelTankVentRBVPin2, &fuelTankVentRBVState, 0); }
    uint32_t stopFuelTankVentRBV(){ stopAct(fuelTankVentRBVPin1, fuelTankVentRBVPin2, &fuelTankVentRBVState, 0); stopFuelTankVentRBVTask->enabled = false; return 0;}
    void brakeFuelTankVentRBV(){ brakeAct(fuelTankVentRBVPin1, fuelTankVentRBVPin2, &fuelTankVentRBVState, 0); }
    void fuelTankVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendFuelTankVentRBV, &retractFuelTankVentRBV, stopFuelTankVentRBVTask); }

    void extendLoxTankVentRBV(){ driveForwards(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
    void retractLoxTankVentRBV(){ driveBackwards(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
    uint32_t stopLoxTankVentRBV(){ stopAct(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); stopLoxTankVentRBVTask->enabled = false; return 0;}
    void brakeLoxTankVentRBV(){ brakeAct(loxTankVentRBVPin1, loxTankVentRBVPin2, &loxTankVentRBVState, 1); }
    void loxTankVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendLoxTankVentRBV, &retractLoxTankVentRBV, stopLoxTankVentRBVTask); }

    void extendIgniterRelay(){ driveForwards(igniterEnablePin1, igniterEnablePin2, &igniterEnableState, 2); }
    void retractIgniterRelay(){ driveBackwards(igniterEnablePin1, igniterEnablePin2, &igniterEnableState, 2); }
    uint32_t stopIgniterRelay(){ stopAct(igniterEnablePin1, igniterEnablePin2, &igniterEnableState, 2); stopIgniterEnableTask->enabled = false; return 0;}
    void brakeIgniterRelay(){ brakeAct(igniterEnablePin1, igniterEnablePin2, &igniterEnableState, 2); }
    void igniterRelayPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendIgniterRelay, &retractIgniterRelay, stopIgniterEnableTask); }

    void extendTwoWay(){ driveForwards(twoWayPin1, twoWayPin2, &twoWayState, 3); }
    void retractTwoWay(){ driveBackwards(twoWayPin1, twoWayPin2, &twoWayState, 3); }
    uint32_t stopTwoWay(){ stopAct(twoWayPin1, twoWayPin2, &twoWayState, 3); stopTwoWayTask->enabled = false; return 0;}
    void brakeTwoWay(){ brakeAct(twoWayPin1, twoWayPin2, &twoWayState, 3); }
    void twoWayPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendTwoWay, &retractTwoWay, stopTwoWayTask); }

    void extendAct5(){ driveForwards(act5Pin1, act5Pin2, &act5State, 4); }
    void retractAct5(){ driveBackwards(act5Pin1, act5Pin2, &act5State, 4); }
    uint32_t stopAct5(){ stopAct(act5Pin1, act5Pin2, &act5State, 4); stop5->enabled = false; return 0;}
    void brakeAct5(){ brakeAct(act5Pin1, act5Pin2, &act5State, 4); }
    void act5PacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendAct5, &retractAct5, stop5); }

    void extendFuelGems(){ driveForwards(fuelGemsPin1, fuelGemsPin2, &fuelGemsState, 5); }
    void retractFuelGems(){ driveBackwards(fuelGemsPin1, fuelGemsPin2, &fuelGemsState, 5); }
    uint32_t stopFuelGems(){ stopAct(fuelGemsPin1, fuelGemsPin2, &fuelGemsState, 5); stopFuelGemsTask->enabled = false; return 0;}
    void brakeFuelGems(){ brakeAct(fuelGemsPin1, fuelGemsPin2, &fuelGemsState, 5); }
    void fuelGemsPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendFuelGems, &retractFuelGems, stopFuelGemsTask); }

    void extendLoxGems(){ driveForwards(loxGemsPin1, loxGemsPin2, &loxGemsState, 6); }
    void retractLoxGems(){ driveBackwards(loxGemsPin1, loxGemsPin2, &loxGemsState, 6); }
    uint32_t stopLoxGems(){ stopAct(loxGemsPin1, loxGemsPin2, &loxGemsState, 6); stopLoxGemsTask->enabled = false; return 0;}
    void brakeLoxGems(){ brakeAct(loxGemsPin1, loxGemsPin2, &loxGemsState, 6); }
    void loxGemsPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendLoxGems, &retractLoxGems, stopLoxGemsTask); }

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
                case 0: stopFuelTankVentRBV(); break;
                case 1: stopLoxTankVentRBV(); break;
                case 2: stopIgniterRelay(); break;
                case 3: stopTwoWay(); break;
                case 4: stopAct5(); break;
                case 5: stopFuelGems(); break;
                case 6: stopLoxGems(); break;
            }
            *actState = 3;
        }

        if ((*actState == 1 || *actState == 2) && *current < stopCurrent){
            switch(actuatorID){
                case 0: stopFuelTankVentRBV(); break;
                case 1: stopLoxTankVentRBV(); break;
                case 2: stopIgniterRelay(); break;
                case 3: stopTwoWay(); break;
                case 4: stopAct5(); break;
                case 5: stopFuelGems(); break;
                case 6: stopLoxGems(); break;
            }
        }

        packet->len = 0;
        Comms::packetAddUint8(packet, *actState);
        Comms::packetAddFloat(packet, *voltage);
        Comms::packetAddFloat(packet, *current);
        Comms::emitPacket(packet);
    }

    uint32_t fuelTankVentRBVSample() {
        sampleActuator(&fuelTankVentRBVPacket, &HAL::chan4, &fuelTankVentRBVVoltage, &fuelTankVentRBVCurrent, &fuelTankVentRBVState, 0);
        return actuatorCheckPeriod;
    }

    uint32_t loxTankVentRBVSample() {
        sampleActuator(&loxTankVentRBVPacket, &HAL::chan5, &loxTankVentRBVVoltage, &loxTankVentRBVCurrent, &loxTankVentRBVState, 1);
        return actuatorCheckPeriod;
    }

    uint32_t igniterEnableSample() {
        sampleActuator(&igniterEnablePacket, &HAL::chan6, &igniterEnableVoltage, &igniterEnableCurrent, &igniterEnableState, 2);
        return actuatorCheckPeriod;
    }

    uint32_t twoWaySample() {
        sampleActuator(&twoWayPacket, &HAL::chan7, &twoWayVoltage, &twoWayCurrent, &twoWayState, 3);
        return actuatorCheckPeriod;
    }

    uint32_t act5Sample() {
        sampleActuator(&act5Packet, &HAL::chan8, &act5Voltage, &act5Current, &act5State, 4);
        return actuatorCheckPeriod;
    }

    uint32_t fuelGemsSample() {
        sampleActuator(&fuelGemsPacket, &HAL::chan9, &fuelGemsVoltage, &fuelGemsCurrent, &fuelGemsState, 5);
        return actuatorCheckPeriod;
    }

    uint32_t loxGemsSample() {
        sampleActuator(&loxGemsPacket, &HAL::chan10, &loxGemsVoltage, &loxGemsCurrent, &loxGemsState, 6);
        return actuatorCheckPeriod;
    }

    void initActuators() {
        Comms::registerCallback(10, fuelTankVentRBVPacketHandler);
        Comms::registerCallback(11, loxTankVentRBVPacketHandler);
        Comms::registerCallback(12, igniterRelayPacketHandler);
        Comms::registerCallback(13, twoWayPacketHandler);
        // Comms::registerCallback(14, act5PacketHandler);
        Comms::registerCallback(15, fuelGemsPacketHandler);
        Comms::registerCallback(16, loxGemsPacketHandler);
    }
};