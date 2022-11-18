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

    void driveForwards(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
        digitalWriteFast(pin1, HIGH);
        digitalWriteFast(pin2, LOW);
        *actState = 1;
    }

    void driveBackwards(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){
        if (channelTypes[actuatorID] == RBV) {
            digitalWriteFast(pin1, LOW);
            digitalWriteFast(pin2, HIGH);
            *actState = 2;
        } else {
            digitalWriteFast(pin1, LOW);
            digitalWriteFast(pin2, LOW);
            *actState = 0;
        }
    }

    void stopAct(uint8_t pin1, uint8_t pin2, uint8_t *actState, uint8_t actuatorID){ //TODO: make it brake
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

    void initActuators() {
        Comms::registerCallback(22, act1PacketHandler);
    }
};