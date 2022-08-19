#include "Actuators.h"

namespace Actuators {

    // TODO: change this to appropriate value
    uint32_t actuatorCheckPeriod = 50 * 1000;

    // TODO: set correct telem packet IDs
    Comms::Packet fuelFillRBVPacket = {.id = 70};
    uint8_t fuelFillRBVState = 0;
    float fuelFillRBVVoltage = 0.0;
    float fuelFillRBVCurrent = 0.0;
    Task *stopFuelFillRBVTask;
    
    Comms::Packet loxFillRBVPacket = {.id = 71};
    uint8_t loxFillRBVState = 0;
    float loxFillRBVVoltage = 0.0;
    float loxFillRBVCurrent = 0.0;
    Task *stopLoxFillRBVTask;

    Comms::Packet pressFillRBVPacket = {.id = 72};
    uint8_t pressFillRBVState = 0;
    float pressFillRBVVoltage = 0.0;
    float pressFillRBVCurrent = 0.0;
    Task *stopPressFillRBVTask;

    Comms::Packet pressLineVentRBVPacket = {.id = 73};
    uint8_t pressLineVentRBVState = 0;
    float pressLineVentRBVVoltage = 0.0;
    float pressLineVentRBVCurrent = 0.0;
    Task *stopPressLineVentRBVTask;

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

    void extendFuelFillRBV(){ driveForwards(fuelFillRBVPin1, fuelFillRBVPin2, &fuelFillRBVState, 0); }
    void retractFuelFillRBV(){ driveBackwards(fuelFillRBVPin1, fuelFillRBVPin2, &fuelFillRBVState, 0); }
    uint32_t stopFuelFillRBV(){ stopAct(fuelFillRBVPin1, fuelFillRBVPin2, &fuelFillRBVState, 0); stopFuelFillRBVTask->enabled = false; return 0;}
    void brakeFuelFillRBV(){ brakeAct(fuelFillRBVPin1, fuelFillRBVPin2, &fuelFillRBVState, 0); }
    void fuelFillRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendFuelFillRBV, &retractFuelFillRBV, stopFuelFillRBVTask); }

    void extendLoxFillRBV(){ driveForwards(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 1); }
    void retractLoxFillRBV(){ driveBackwards(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 1); }
    uint32_t stopLoxFillRBV(){ stopAct(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 1); stopLoxFillRBVTask->enabled = false; return 0;}
    void brakeLoxFillRBV(){ brakeAct(loxFillRBVPin1, loxFillRBVPin2, &loxFillRBVState, 1); }
    void loxFillRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendLoxFillRBV, &retractLoxFillRBV, stopLoxFillRBVTask); }

    void extendPressFillRBV(){ driveForwards(pressFillRBVPin1, pressFillRBVPin2, &pressFillRBVState, 2); }
    void retractPressFillRBV(){ driveBackwards(pressFillRBVPin1, pressFillRBVPin2, &pressFillRBVState, 2); }
    uint32_t stopPressFillRBV(){ stopAct(pressFillRBVPin1, pressFillRBVPin2, &pressFillRBVState, 2); stopPressFillRBVTask->enabled = false; return 0;}
    void brakePressFillRBV(){ brakeAct(pressFillRBVPin1, pressFillRBVPin2, &pressFillRBVState, 2); }
    void pressFillRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendPressFillRBV, &retractPressFillRBV, stopPressFillRBVTask); }

    void extendPressLineVentRBV(){ driveForwards(pressLineVentRBVPin1, pressLineVentRBVPin2, &pressLineVentRBVState, 3); }
    void retractPressLineVentRBV(){ driveBackwards(pressLineVentRBVPin1, pressLineVentRBVPin2, &pressLineVentRBVState, 3); }
    uint32_t stopPressLineVentRBV(){ stopAct(pressLineVentRBVPin1, pressLineVentRBVPin2, &pressLineVentRBVState, 3); stopPressLineVentRBVTask->enabled = false; return 0;}
    void brakePressLineVentRBV(){ brakeAct(pressLineVentRBVPin1, pressLineVentRBVPin2, &pressLineVentRBVState, 3); }
    void pressLineVentRBVPacketHandler(Comms::Packet tmp, uint8_t ip){ actPacketHandler(tmp, &extendPressLineVentRBV, &retractPressLineVentRBV, stopPressLineVentRBVTask); }

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
                case 0: stopFuelFillRBV(); break;
                case 1: stopLoxFillRBV(); break;
                case 2: stopPressFillRBV(); break;
                case 3: stopPressLineVentRBV(); break;
                case 4: stopAct5(); break;
                case 5: stopAct6(); break;
                case 6: stopAct7(); break;
            }
            *actState = 3;
        }

        if ((*actState == 1 || *actState == 2) && *current < stopCurrent){
            switch(actuatorID){
                case 0: stopFuelFillRBV(); break;
                case 1: stopLoxFillRBV(); break;
                case 2: stopPressFillRBV(); break;
                case 3: stopPressLineVentRBV(); break;
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

    uint32_t fuelFillRBVSample() {
        sampleActuator(&fuelFillRBVPacket, &HAL::chan4, &fuelFillRBVVoltage, &fuelFillRBVCurrent, &fuelFillRBVState, 0);
        return actuatorCheckPeriod;
    }

    uint32_t loxFillRBVSample() {
        sampleActuator(&loxFillRBVPacket, &HAL::chan5, &loxFillRBVVoltage, &loxFillRBVCurrent, &loxFillRBVState, 1);
        return actuatorCheckPeriod;
    }

    uint32_t pressFillRBVSample() {
        sampleActuator(&pressFillRBVPacket, &HAL::chan6, &pressFillRBVVoltage, &pressFillRBVCurrent, &pressFillRBVState, 2);
        return actuatorCheckPeriod;
    }

    uint32_t pressLineVentRBVSample() {
        sampleActuator(&pressLineVentRBVPacket, &HAL::chan7, &pressLineVentRBVVoltage, &pressLineVentRBVCurrent, &pressLineVentRBVState, 3);
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
        Comms::registerCallback(10, fuelFillRBVPacketHandler);
        Comms::registerCallback(11, loxFillRBVPacketHandler);
        Comms::registerCallback(12, pressFillRBVPacketHandler);
        Comms::registerCallback(13, pressLineVentRBVPacketHandler);
        // Comms::registerCallback(14, act5PacketHandler);
        // Comms::registerCallback(15, act6PacketHandler);
        // Comms::registerCallback(16, act7PacketHandler);
    }
};